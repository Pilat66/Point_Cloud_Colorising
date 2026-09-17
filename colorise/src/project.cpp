#include "colorise.h"

#include <cmath>
#include <cstdint>

// ─────────────────────────────────────────────────────────────────────────────
// Projection core — a 1:1 port of project_and_sample() from colorise_offline.py
// ─────────────────────────────────────────────────────────────────────────────

void projectAndSample(const Cloud& cloud, const std::vector<int>& cand,
                      const cv::Mat& img, const CameraParams& cam,
                      const Eigen::Matrix4d& T_cam_from_world,
                      double edge_margin, double max_view_angle_deg,
                      double min_camera_dist, bool occlusion,
                      double occlusion_cell_px, double occlusion_depth_tol,
                      std::vector<uint32_t>& out_rgb, std::vector<int>& out_idx,
                      std::vector<double>& out_dist) {
    out_rgb.clear();
    out_idx.clear();
    out_dist.clear();

    const int H = img.rows;
    const int W = img.cols;
    if (cand.empty() || img.empty() || H <= 0 || W <= 0) return;

    const Eigen::Matrix3d R = T_cam_from_world.topLeftCorner(3, 3);
    const Eigen::Vector3d t = T_cam_from_world.col(3).head(3);

    // Pass 1: geometry gates (z>0, d>0, min-camera-dist, max view angle).
    std::vector<cv::Point3f> cam_pts;
    std::vector<int> cam_idx;
    std::vector<double> depth;                    // pc.z, doubles (Python float64)
    std::vector<double> dists;                    // |pc| — camera-to-point distance
    cam_pts.reserve(cand.size());
    cam_idx.reserve(cand.size());
    depth.reserve(cand.size());
    dists.reserve(cand.size());

    const double cos_view = (max_view_angle_deg < 180.0)
                                ? std::cos(max_view_angle_deg * M_PI / 180.0)
                                : -1.0;

    for (size_t k = 0; k < cand.size(); ++k) {
        const int i = cand[k];
        const Eigen::Vector3d pw(cloud.x[i], cloud.y[i], cloud.z[i]);
        const Eigen::Vector3d pc = R * pw + t;
        const double z = pc.z();
        const double d3 = pc.norm();
        if (!(z > 1e-6) || !(d3 > 1e-6)) continue;
        if (min_camera_dist > 0.0 && d3 < min_camera_dist) continue;
        if (cos_view > -1.0 && z / d3 < cos_view) continue;
        cam_pts.emplace_back(static_cast<float>(pc.x()),
                             static_cast<float>(pc.y()),
                             static_cast<float>(pc.z()));
        cam_idx.push_back(i);
        depth.push_back(z);
        dists.push_back(d3);
    }
    if (cam_pts.empty()) return;

    // Pass 2: project. Points are already in the camera frame, and the Python
    // script also passes zero rvec/tvec; the fisheye model is used when the
    // camera says so (same fallback logic as Python's hasattr(cv2,"fisheye")).
    const cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64F);
    const cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64F);
    cv::Mat K = cv::Mat::zeros(3, 3, CV_64F);
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            K.at<double>(r, c) = cam.K(r, c);
    cv::Mat D(1, static_cast<int>(cam.dist.size()), CV_64F);
    for (size_t i = 0; i < cam.dist.size(); ++i)
        D.at<double>(0, static_cast<int>(i)) = cam.dist[i];

    std::vector<cv::Point2f> P2;
    if (cam.model == "fisheye")
        // fisheye::projectPoints(obj, imagePoints, rvec, tvec, K, D)
        cv::fisheye::projectPoints(cam_pts, P2, rvec, tvec, K, D);
    else
        // projectPoints(obj, rvec, tvec, K, D, imagePoints)
        cv::projectPoints(cam_pts, rvec, tvec, K, D, P2);

    // Pass 2b: usable rectangle after trimming edge_margin; np.round -> nearbyint.
    const int m = static_cast<int>(std::lround(edge_margin));
    const int u_lo = m, u_hi = cam.width - m;
    const int v_lo = m, v_hi = cam.height - m;

    std::vector<int> uu(P2.size()), vv(P2.size());
    std::vector<unsigned char> ok(P2.size(), 0);
    for (size_t k = 0; k < P2.size(); ++k) {
        const double px = P2[k].x, py = P2[k].y;
        if (!std::isfinite(px) || !std::isfinite(py)) continue;
        const int u = static_cast<int>(std::nearbyint(px));
        const int v = static_cast<int>(std::nearbyint(py));
        if (u < u_lo || u >= u_hi || v < v_lo || v >= v_hi) continue;
        uu[k] = u;
        vv[k] = v;
        ok[k] = 1;
    }
// Pass 3: z-buffer occlusion (Python np.minimum.at over grid cells).
    if (occlusion) {
        const double cell = std::max(1.0, occlusion_cell_px);
        const int gw = static_cast<int>(std::ceil(cam.width / cell));
        const int gh = static_cast<int>(std::ceil(cam.height / cell));
        std::vector<double> zmin(static_cast<size_t>(gw) * gh,
                                 std::numeric_limits<double>::infinity());
        for (size_t k = 0; k < P2.size(); ++k) {
            if (!ok[k]) continue;
            const int gx = static_cast<int>(uu[k] / cell);
            const int gy = static_cast<int>(vv[k] / cell);
            double& z = zmin[static_cast<size_t>(gy) * gw + gx];
            if (depth[k] < z) z = depth[k];
        }
        for (size_t k = 0; k < P2.size(); ++k) {
            if (!ok[k]) continue;
            const int gx = static_cast<int>(uu[k] / cell);
            const int gy = static_cast<int>(vv[k] / cell);
            const double zmin_cell = zmin[static_cast<size_t>(gy) * gw + gx];
            if (depth[k] > zmin_cell + occlusion_depth_tol) ok[k] = 0;
        }
    }

    // Pass 4: sample colours from the BGR image and pack PCL-style rgb.
    out_rgb.reserve(P2.size());
    out_idx.reserve(P2.size());
    for (size_t k = 0; k < P2.size(); ++k) {
        if (!ok[k]) continue;
        int u = uu[k], v = vv[k];
        if (W != cam.width || H != cam.height) {
            u = static_cast<int>(static_cast<double>(u) *
                                 (static_cast<double>(W) / cam.width));
            v = static_cast<int>(static_cast<double>(v) *
                                 (static_cast<double>(H) / cam.height));
        }
        u = std::max(0, std::min(u, W - 1));
        v = std::max(0, std::min(v, H - 1));
        const cv::Vec3b bgr = img.at<cv::Vec3b>(v, u);
        const uint32_t packed = (static_cast<uint32_t>(bgr[2]) << 16) |
                                (static_cast<uint32_t>(bgr[1]) << 8) |
                                static_cast<uint32_t>(bgr[0]);
        out_rgb.push_back(packed);
        out_idx.push_back(cam_idx[k]);
        out_dist.push_back(dists[k]);
    }
}