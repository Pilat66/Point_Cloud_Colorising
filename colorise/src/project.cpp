#include "colorise.h"

#include <cmath>
#include <cstdint>
#include <cstring>

// ─────────────────────────────────────────────────────────────────────────────
// Projection core — a 1:1 port of project_and_sample() from colorise_offline.py
// ─────────────────────────────────────────────────────────────────────────────

// Аппроксимация sqrt(d2) для метрики --nearest-wins: бит-трюк fast inverse
// sqrt (float32) + одна итерация Ньютона, относительная ошибка <0.2% на любой
// дистанции. Арифметика бит-в-бит совпадает с батчевым расчётом в
// colorise_offline.py (project_and_sample) — паритет C++/Python.
static double fastSqrtApprox(double d2) {
    float f = static_cast<float>(d2);
    std::uint32_t i;
    std::memcpy(&i, &f, sizeof i);
    i = 0x5f3759dfu - (i >> 1);
    float y;
    std::memcpy(&y, &i, sizeof y);
    y = y * (1.5f - 0.5f * f * y * y);
    return static_cast<double>(f * y);
}

// Собственная векторная проекция pinhole (radtan/plumb_bob): x/z, дисторсия
// k1,k2,p1,p2,k3, затем fx,fy,cx,cy. Арифметика и порядок операций совпадают с
// project_pinhole() в colorise_offline.py — паритет C++/Python байт-в-байт.
// Применяется, когда модель pinhole и коэффициентов не больше 5; иначе — OpenCV.
static void projectPinhole(const std::vector<Eigen::Vector3d>& pts,
                           const CameraParams& cam,
                           std::vector<double>& u_out,
                           std::vector<double>& v_out) {
    u_out.resize(pts.size());
    v_out.resize(pts.size());
    const double fx = cam.K(0, 0), cx = cam.K(0, 2);
    const double fy = cam.K(1, 1), cy = cam.K(1, 2);
    double k1 = 0.0, k2 = 0.0, p1 = 0.0, p2 = 0.0, k3 = 0.0;
    if (cam.dist.size() > 0) k1 = cam.dist[0];
    if (cam.dist.size() > 1) k2 = cam.dist[1];
    if (cam.dist.size() > 2) p1 = cam.dist[2];
    if (cam.dist.size() > 3) p2 = cam.dist[3];
    if (cam.dist.size() > 4) k3 = cam.dist[4];
    for (std::size_t i = 0; i < pts.size(); ++i) {
        const double X = pts[i].x(), Y = pts[i].y(), Z = pts[i].z();
        const double x = X / Z, y = Y / Z;
        const double r2 = x * x + y * y;
        const double radial = 1.0 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2;
        const double xd = x * radial + 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x);
        const double yd = y * radial + p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y;
        u_out[i] = fx * xd + cx;
        v_out[i] = fy * yd + cy;
    }
}

void projectAndSample(const Cloud& cloud, const std::vector<int>& cand,
                      const cv::Mat& img, const CameraParams& cam,
                      const Eigen::Matrix4d& T_cam_from_world,
                      double edge_margin, double max_view_angle_deg,
                      double min_camera_dist, bool occlusion,
                      double occlusion_cell_px, double occlusion_depth_tol,
                      double occlusion_max_depth, int occlusion_ray_margin,
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
    std::vector<Eigen::Vector3d> cam_pts;         // camera-frame points (doubles)
    std::vector<int> cam_idx;
    std::vector<double> depth;                    // pc.z, doubles (Python float64)
    std::vector<double> dists;                    // approx |pc| (~0.2%), --nearest-wins
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
        cam_pts.emplace_back(pc.x(), pc.y(), pc.z());
        cam_idx.push_back(i);
        depth.push_back(z);
        dists.push_back(fastSqrtApprox(pc.squaredNorm()));
    }
    if (cam_pts.empty()) return;

    // Pass 2: project. Points are already in the camera frame, and the Python
    // script also passes zero rvec/tvec. Pinhole with <=5 distortion coefficients
    // uses our own vectorised projection (like project_pinhole() in Python);
    // fisheye and longer coefficient vectors fall back to OpenCV, exactly as the
    // Python script does.
    const bool own_pinhole =
        (cam.model != "fisheye") && (cam.dist.size() <= 5);
    std::vector<double> u_d, v_d;                 // own projection (doubles)
    std::vector<cv::Point2f> P2;                  // OpenCV projection
    if (own_pinhole) {
        projectPinhole(cam_pts, cam, u_d, v_d);
    } else {
        std::vector<cv::Point3f> pts_f;
        pts_f.reserve(cam_pts.size());
        for (const Eigen::Vector3d& p : cam_pts)
            pts_f.emplace_back(static_cast<float>(p.x()),
                               static_cast<float>(p.y()),
                               static_cast<float>(p.z()));
        const cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64F);
        const cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64F);
        cv::Mat K = cv::Mat::zeros(3, 3, CV_64F);
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
                K.at<double>(r, c) = cam.K(r, c);
        cv::Mat D(1, static_cast<int>(cam.dist.size()), CV_64F);
        for (size_t i = 0; i < cam.dist.size(); ++i)
            D.at<double>(0, static_cast<int>(i)) = cam.dist[i];
        if (cam.model == "fisheye")
            // fisheye::projectPoints(obj, imagePoints, rvec, tvec, K, D)
            cv::fisheye::projectPoints(pts_f, P2, rvec, tvec, K, D);
        else
            // projectPoints(obj, rvec, tvec, K, D, imagePoints)
            cv::projectPoints(pts_f, rvec, tvec, K, D, P2);
    }

    // Pass 2b: usable rectangle after trimming edge_margin; np.round -> nearbyint.
    const int m = static_cast<int>(std::lround(edge_margin));
    const int u_lo = m, u_hi = cam.width - m;
    const int v_lo = m, v_hi = cam.height - m;

    const std::size_t npts = own_pinhole ? u_d.size() : P2.size();
    std::vector<int> uu(npts), vv(npts);
    std::vector<unsigned char> ok(npts, 0);
    for (size_t k = 0; k < npts; ++k) {
        const double px = own_pinhole ? u_d[k] : static_cast<double>(P2[k].x);
        const double py = own_pinhole ? v_d[k] : static_cast<double>(P2[k].y);
        if (!std::isfinite(px) || !std::isfinite(py)) continue;
        const int u = static_cast<int>(std::nearbyint(px));
        const int v = static_cast<int>(std::nearbyint(py));
        if (u < u_lo || u >= u_hi || v < v_lo || v >= v_hi) continue;
        uu[k] = u;
        vv[k] = v;
        ok[k] = 1;
    }
// Pass 3: z-buffer — occlusion rejection plus the "no lidar rays" gate.
    // The grid is built whenever either check is active.
    const bool no_ray_gate = std::isfinite(occlusion_max_depth);
    if (occlusion || no_ray_gate) {
        const double cell = std::max(1.0, occlusion_cell_px);
        const int gw = static_cast<int>(std::ceil(cam.width / cell));
        const int gh = static_cast<int>(std::ceil(cam.height / cell));
        const std::size_t ncells =
            static_cast<std::size_t>(gw) * static_cast<std::size_t>(gh);
        std::vector<double> zmin(ncells,
                                 std::numeric_limits<double>::infinity());
        auto cellOf = [&](std::size_t k) {
            const int gx = static_cast<int>(uu[k] / cell);
            const int gy = static_cast<int>(vv[k] / cell);
            return static_cast<std::size_t>(gy) * gw + gx;
        };
        for (size_t k = 0; k < npts; ++k) {
            if (!ok[k]) continue;
            double& z = zmin[cellOf(k)];
            if (depth[k] < z) z = depth[k];
        }
        if (occlusion) {
            for (size_t k = 0; k < npts; ++k) {
                if (!ok[k]) continue;
                if (depth[k] > zmin[cellOf(k)] + occlusion_depth_tol) ok[k] = 0;
            }
        }
        // "No lidar rays": a cell whose minimum depth is beyond the limit — or
        // that received no point at all (min = +inf) — holds no lidar rays. The
        // mask is dilated by occlusion_ray_margin cells (Chebyshev metric) so
        // points next to such an area are dropped too, and any point landing in
        // the dilated mask is not coloured.
        if (no_ray_gate) {
            std::vector<unsigned char> noray(ncells, 0);
            for (std::size_t c = 0; c < ncells; ++c)
                if (zmin[c] > occlusion_max_depth) noray[c] = 1;
            if (occlusion_ray_margin > 0) {
                const int N = occlusion_ray_margin;
                std::vector<unsigned char> h(ncells, 0), v(ncells, 0);
                for (int gy = 0; gy < gh; ++gy)
                    for (int gx = 0; gx < gw; ++gx) {
                        if (!noray[static_cast<std::size_t>(gy) * gw + gx])
                            continue;
                        const int x0 = std::max(0, gx - N);
                        const int x1 = std::min(gw - 1, gx + N);
                        for (int x = x0; x <= x1; ++x)
                            h[static_cast<std::size_t>(gy) * gw + x] = 1;
                    }
                for (int gx = 0; gx < gw; ++gx)
                    for (int gy = 0; gy < gh; ++gy) {
                        if (!h[static_cast<std::size_t>(gy) * gw + gx]) continue;
                        const int y0 = std::max(0, gy - N);
                        const int y1 = std::min(gh - 1, gy + N);
                        for (int y = y0; y <= y1; ++y)
                            v[static_cast<std::size_t>(y) * gw + gx] = 1;
                    }
                noray.swap(v);
            }
            for (size_t k = 0; k < npts; ++k) {
                if (!ok[k]) continue;
                if (noray[cellOf(k)]) ok[k] = 0;
            }
        }
    }

    // Pass 4: sample colours from the BGR image and pack PCL-style rgb.
    out_rgb.reserve(npts);
    out_idx.reserve(npts);
    out_dist.reserve(npts);
    for (size_t k = 0; k < npts; ++k) {
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