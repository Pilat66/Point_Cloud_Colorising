#include "colorise.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>

// ─────────────────────────────────────────────────────────────────────────────
// Compensation mode
// ─────────────────────────────────────────────────────────────────────────────

CompMode parseCompMode(const std::string& s) {
    if (s == "none" || s == "off")   return CompMode::NONE;
    if (s == "imu")                  return CompMode::IMU;
    if (s == "odom") return CompMode::ODOM;
    throw std::runtime_error("Unknown compensation_mode '" + s +
                             "' (expected: none | imu | odom)");
}

std::string compModeName(CompMode m) {
    switch (m) {
        case CompMode::NONE: return "none";
        case CompMode::IMU:  return "imu";
        case CompMode::ODOM: return "odom";
    }
    return "?";
}

// ─────────────────────────────────────────────────────────────────────────────
// Parameter loading
// ─────────────────────────────────────────────────────────────────────────────

namespace {

template <typename T>
T get(const YAML::Node& n, const std::string& key, const T& fallback) {
    return n[key] ? n[key].as<T>() : fallback;
}

template <typename T>
T require(const YAML::Node& n, const std::string& key) {
    if (!n[key]) throw std::runtime_error("Missing required config key: " + key);
    return n[key].as<T>();
}

}  // namespace

void loadCommonParams(const std::string& config_path, CommonParams& p) {
    YAML::Node cfg = loadYamlLenient(config_path);

    p.calibration_path = expandRosPath(require<std::string>(cfg, "calibration_path"));
    YAML::Node calib   = loadYamlLenient(p.calibration_path);

    p.pointcloud_topic = get<std::string>(cfg, "pointcloud_topic", "/ouster/points");
    p.imu_topic        = get<std::string>(cfg, "imu_topic",        "/ms/imu/data");
    p.odom_topic       = get<std::string>(cfg, "odom_topic",       "/lio_sam/mapping/odometry");
    p.output_topic     = get<std::string>(cfg, "output_topic",     "/colorised_points");

    p.max_time_offset       = get<double>(cfg, "max_time_offset",       0.05);
    p.initial_startup_delay = get<double>(cfg, "initial_startup_delay", 0.1);
    p.keep_uncolored_points = get<bool>  (cfg, "keep_uncolored_points", false);
    p.max_lidar_z           = get<double>(cfg, "max_lidar_z",           100.0);
    p.compensation_mode     = parseCompMode(get<std::string>(cfg, "compensation_mode", "imu"));

    p.occlusion_check     = get<bool>  (cfg, "occlusion_check",     true);
    p.occlusion_cell_px   = get<double>(cfg, "occlusion_cell_px",   4.0);
    p.occlusion_depth_tol = get<double>(cfg, "occlusion_depth_tol", 0.3);
    p.occlusion_max_depth  = get<double>(cfg, "occlusion_max_depth",
                                         std::numeric_limits<double>::infinity());
    p.occlusion_ray_margin = get<int>   (cfg, "occlusion_ray_margin", 1);

    p.map_pcd_path     = expandRosPath(get<std::string>(cfg, "map_pcd_path",  ""));
    p.odom_csv_path    = expandRosPath(get<std::string>(cfg, "odom_csv_path", ""));
    p.save_pcd_path    = expandRosPath(get<std::string>(cfg, "save_pcd_path", ""));
    p.min_color_frames = get<int>   (cfg, "min_color_frames", 1);
    p.map_max_range    = get<double>(cfg, "map_max_range",    30.0);
    p.map_publish_every_n = std::max(1, get<int>(cfg, "map_publish_every_n", 5));

    // IMU-to-lidar rotation, used by CompMode::IMU.
    std::string imu_key = get<std::string>(cfg, "imu_extrinsic_key", "T_imu_link_os_sensor");
    p.R_imu_lidar = parseMatrix4d(calib, imu_key).block<3, 3>(0, 0);

    // Global fisheye edge-rejection defaults; cameras may override.
    double default_margin = get<double>(cfg, "edge_margin_px",     0.0);
    double default_angle  = get<double>(cfg, "max_view_angle_deg", 180.0);
    double default_near   = get<double>(cfg, "min_camera_dist",    0.0);

    if (!cfg["cameras"] || !cfg["cameras"].IsSequence() || cfg["cameras"].size() == 0)
        throw std::runtime_error("config.yaml must define a non-empty 'cameras' list");

    for (const auto& c : cfg["cameras"]) {
        CameraCalib cam;
        cam.name        = require<std::string>(c, "name");
        cam.image_topic = require<std::string>(c, "image_topic");

        // Extrinsic. Kalibr's T_A_B is "A from B", so T_os_sensor_<cam> is
        // lidar-from-camera and must be inverted for projection.
        std::string ext_key = require<std::string>(c, "extrinsic_key");
        bool lidar_from_cam = get<bool>(c, "extrinsic_is_lidar_from_cam", true);
        Eigen::Matrix4d T  = parseMatrix4d(calib, ext_key);
        cam.T_lidar_cam    = lidar_from_cam ? T : T.inverse();
        cam.T_cam_lidar    = cam.T_lidar_cam.inverse();

        // Intrinsics + distortion, straight from calibration.yaml.
        std::string ikey = require<std::string>(c, "intrinsics_key");
        if (!calib[ikey])
            throw std::runtime_error("Missing intrinsics block '" + ikey + "' in calibration file");
        const YAML::Node& ic = calib[ikey];

        auto intr = ic["intrinsics"].as<std::vector<double>>();
        if (intr.size() != 4)
            throw std::runtime_error("'" + ikey + ".intrinsics' must be [fx, fy, cx, cy]");
        cam.K = (cv::Mat_<double>(3, 3) << intr[0], 0.0,     intr[2],
                                           0.0,     intr[1], intr[3],
                                           0.0,     0.0,     1.0);

        auto dist = ic["distortion_coeffs"].as<std::vector<double>>();
        cam.D = cv::Mat(1, static_cast<int>(dist.size()), CV_64F);
        for (size_t i = 0; i < dist.size(); ++i)
            cam.D.at<double>(0, static_cast<int>(i)) = dist[i];

        cam.distortion_model = ic["distortion_model"].as<std::string>();
        auto res = ic["resolution"].as<std::vector<int>>();
        if (res.size() != 2)
            throw std::runtime_error("'" + ikey + ".resolution' must be [width, height]");
        cam.width  = res[0];
        cam.height = res[1];

        if (cam.distortion_model == "equidistant" && cam.D.cols != 4)
            throw std::runtime_error("'" + ikey + "' is equidistant (fisheye) so it needs "
                                     "exactly 4 distortion coefficients, got " +
                                     std::to_string(cam.D.cols));

        cam.edge_margin_px     = get<double>(c, "edge_margin_px",     default_margin);
        cam.max_view_angle_deg = get<double>(c, "max_view_angle_deg", default_angle);
        cam.min_camera_dist    = get<double>(c, "min_camera_dist",    default_near);
        cam.min_cos_view_angle = (cam.max_view_angle_deg >= 180.0)
                                   ? -1.0
                                   : std::cos(cam.max_view_angle_deg * M_PI / 180.0);

        if (2.0 * cam.edge_margin_px >= std::min(cam.width, cam.height))
            throw std::runtime_error("edge_margin_px (" +
                                     std::to_string(cam.edge_margin_px) +
                                     ") leaves no usable image area for camera " + cam.name);

        p.cameras.push_back(cam);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Projection core
// ─────────────────────────────────────────────────────────────────────────────

void projectAndSample(const std::vector<cv::Point3f>& P3,
                      const std::vector<int>&         candidates,
                      const cv::Mat&                  img,
                      const CameraCalib&              cam,
                      const Eigen::Matrix4d&          T_cam_from_points,
                      bool                            occlusion_check,
                      double                          occlusion_cell_px,
                      double                          occlusion_depth_tol,
                      double                          occlusion_max_depth,
                      int                             occlusion_ray_margin,
                      const std::function<void(int, std::uint32_t)>& sink) {
    if (img.empty()) return;

    const Eigen::Matrix3d R = T_cam_from_points.block<3, 3>(0, 0);
    const Eigen::Vector3d t = T_cam_from_points.block<3, 1>(0, 3);

    const size_t n = candidates.size();
    if (n == 0) return;

    // Pass 1: move points into the camera frame and drop everything that cannot
    // possibly be seen — behind the camera, or outside the usable fisheye cone.
    // Doing this before projection means cv::fisheye::projectPoints never sees
    // the points where the equidistant model folds back on itself.
    std::vector<cv::Point3f> cam_pts;
    std::vector<int>         cam_idx;
    std::vector<float>       depths;
    cam_pts.reserve(n);
    cam_idx.reserve(n);
    depths.reserve(n);

    for (size_t k = 0; k < n; ++k) {
        const int i = candidates[k];
        const Eigen::Vector3d pw(P3[i].x, P3[i].y, P3[i].z);
        const Eigen::Vector3d pc = R * pw + t;

        if (pc.z() <= 0.0) continue;  // behind the camera

        const double dist = pc.norm();
        if (dist < 1e-6) continue;

        // Near-range gate. Distance from the camera centre, not depth along the
        // optical axis: on a fisheye a point 1 m off to the side is just as
        // close, and just as badly modelled, as one 1 m straight ahead.
        if (dist < cam.min_camera_dist) continue;

        // cos(angle from optical axis) = z / |p|
        if (cam.min_cos_view_angle > -1.0 &&
            pc.z() / dist < cam.min_cos_view_angle) continue;

        cam_pts.emplace_back(static_cast<float>(pc.x()),
                             static_cast<float>(pc.y()),
                             static_cast<float>(pc.z()));
        cam_idx.push_back(i);
        depths.push_back(static_cast<float>(pc.z()));
    }
    if (cam_pts.empty()) return;

    // Pass 2: project. Points are already in the camera frame, so the extrinsic
    // passed to OpenCV is the identity.
    const cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64F);
    const cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64F);

    std::vector<cv::Point2f> P2;
    if (cam.distortion_model == "equidistant")
        cv::fisheye::projectPoints(cam_pts, P2, rvec, tvec, cam.K, cam.D);
    else
        cv::projectPoints(cam_pts, P2, rvec, tvec, cam.K, cam.D);

    // Usable image rectangle after trimming the distorted fisheye border.
    const int m    = static_cast<int>(std::lround(cam.edge_margin_px));
    const int u_lo = m,                 u_hi = cam.width  - m;
    const int v_lo = m,                 v_hi = cam.height - m;

    std::vector<int> uu(P2.size()), vv(P2.size());
    std::vector<bool> ok(P2.size(), false);

    for (size_t k = 0; k < P2.size(); ++k) {
        if (!std::isfinite(P2[k].x) || !std::isfinite(P2[k].y)) continue;
        const int u = static_cast<int>(std::lround(P2[k].x));
        const int v = static_cast<int>(std::lround(P2[k].y));
        if (u < u_lo || u >= u_hi || v < v_lo || v >= v_hi) continue;
        uu[k] = u;
        vv[k] = v;
        ok[k] = true;
    }

    // Pass 3: z-buffer. Without this, a wall and everything behind it land on
    // the same pixels and all get painted with the wall's colour. The grid is
    // also the source of the "no lidar rays" mask (see below).
    const bool no_ray_gate = std::isfinite(occlusion_max_depth);
    if (occlusion_check || no_ray_gate) {
        const double cell = std::max(1.0, occlusion_cell_px);
        const int gw = static_cast<int>(std::ceil(cam.width  / cell));
        const int gh = static_cast<int>(std::ceil(cam.height / cell));
        const std::size_t ncells =
            static_cast<std::size_t>(gw) * static_cast<std::size_t>(gh);
        std::vector<float> zbuf(ncells, std::numeric_limits<float>::infinity());
        auto cellOf = [&](std::size_t k) {
            const int gx = static_cast<int>(uu[k] / cell);
            const int gy = static_cast<int>(vv[k] / cell);
            return static_cast<std::size_t>(gy) * gw + gx;
        };

        for (size_t k = 0; k < P2.size(); ++k) {
            if (!ok[k]) continue;
            float& z = zbuf[cellOf(k)];
            z = std::min(z, depths[k]);
        }
        if (occlusion_check) {
            for (size_t k = 0; k < P2.size(); ++k) {
                if (!ok[k]) continue;
                const float zmin = zbuf[cellOf(k)];
                if (depths[k] > zmin + occlusion_depth_tol) ok[k] = false;
            }
        }
        // "No lidar rays": a cell whose minimum depth is beyond the limit — or
        // that received no point at all (min = +inf) — holds no lidar rays. The
        // mask is dilated by occlusion_ray_margin cells (Chebyshev metric) and
        // any point landing in the dilated mask is not coloured.
        if (no_ray_gate) {
            std::vector<unsigned char> noray(ncells, 0);
            for (std::size_t c = 0; c < ncells; ++c)
                if (static_cast<double>(zbuf[c]) > occlusion_max_depth)
                    noray[c] = 1;
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
            for (size_t k = 0; k < P2.size(); ++k) {
                if (!ok[k]) continue;
                if (noray[cellOf(k)]) ok[k] = false;
            }
        }
    }

    // Pass 4: sample colour. The image may be at a different resolution than
    // the calibration; scale pixel coordinates rather than reading out of range.
    const double sx = static_cast<double>(img.cols) / cam.width;
    const double sy = static_cast<double>(img.rows) / cam.height;

    for (size_t k = 0; k < P2.size(); ++k) {
        if (!ok[k]) continue;
        int iu = uu[k], iv = vv[k];
        if (sx != 1.0 || sy != 1.0) {
            iu = static_cast<int>(uu[k] * sx);
            iv = static_cast<int>(vv[k] * sy);
        }
        if (iu < 0 || iu >= img.cols || iv < 0 || iv >= img.rows) continue;

        const cv::Vec3b c = img.at<cv::Vec3b>(iv, iu);
        const std::uint32_t rgb = (static_cast<std::uint32_t>(c[2]) << 16) |
                                  (static_cast<std::uint32_t>(c[1]) <<  8) |
                                  (static_cast<std::uint32_t>(c[0]));
        sink(cam_idx[k], rgb);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// IMU buffer
// ─────────────────────────────────────────────────────────────────────────────

void ImuBuffer::push(const sensor_msgs::ImuConstPtr& msg) {
    buf_.push_back(msg);
    while (!buf_.empty() &&
           (msg->header.stamp - buf_.front()->header.stamp).toSec() > history_sec_)
        buf_.pop_front();
}

bool ImuBuffer::covers(const ros::Time& t) const {
    return buf_.size() >= 2 &&
           t >= buf_.front()->header.stamp &&
           t <= buf_.back()->header.stamp;
}

bool ImuBuffer::interpolate(const ros::Time& t, Eigen::Quaterniond& q_out) const {
    if (buf_.size() < 2) return false;
    if (t < buf_.front()->header.stamp || t > buf_.back()->header.stamp) return false;

    // Binary search: the buffer is time-ordered and holds ~2000 samples at 200 Hz.
    auto it = std::lower_bound(buf_.begin(), buf_.end(), t,
        [](const sensor_msgs::ImuConstPtr& m, const ros::Time& target) {
            return m->header.stamp < target;
        });

    if (it == buf_.begin()) {
        const auto& q = (*it)->orientation;
        q_out = Eigen::Quaterniond(q.w, q.x, q.y, q.z).normalized();
        return true;
    }

    auto hi = it;
    auto lo = std::prev(it);

    const double dt = ((*hi)->header.stamp - (*lo)->header.stamp).toSec();
    const double a  = (dt > 1e-9) ? (t - (*lo)->header.stamp).toSec() / dt : 0.0;

    const auto& q0 = (*lo)->orientation;
    const auto& q1 = (*hi)->orientation;
    q_out = Eigen::Quaterniond(q0.w, q0.x, q0.y, q0.z).normalized()
                .slerp(a, Eigen::Quaterniond(q1.w, q1.x, q1.y, q1.z).normalized());
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Odometry trajectory
// ─────────────────────────────────────────────────────────────────────────────

Eigen::Matrix4d makeTransform(const Eigen::Quaterniond& q, const Eigen::Vector3d& p) {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = q.normalized().toRotationMatrix();
    T.block<3, 1>(0, 3) = p;
    return T;
}

size_t OdomTrajectory::loadCsv(const std::string& path) {
    std::ifstream f(path);
    if (!f.is_open())
        throw std::runtime_error("Cannot open odometry CSV: " + path);

    samples_.clear();
    live_ = false;

    std::string line;
    size_t line_no = 0;
    while (std::getline(f, line)) {
        ++line_no;
        if (line.empty() || line[0] == '#') continue;
        // Skip a header row if present.
        if (line_no == 1 && (line.find("time") != std::string::npos ||
                             line.find("stamp") != std::string::npos))
            continue;

        std::stringstream ss(line);
        std::string tok;
        std::vector<double> v;
        while (std::getline(ss, tok, ',')) {
            try { v.push_back(std::stod(tok)); }
            catch (...) { v.clear(); break; }
        }
        if (v.size() < 8) continue;

        Sample s;
        s.t = ros::Time(v[0]);
        s.p = Eigen::Vector3d(v[1], v[2], v[3]);
        s.q = Eigen::Quaterniond(v[7], v[4], v[5], v[6]).normalized();  // w,x,y,z
        samples_.push_back(s);
    }

    std::sort(samples_.begin(), samples_.end(),
              [](const Sample& a, const Sample& b) { return a.t < b.t; });

    if (samples_.size() < 2)
        throw std::runtime_error("Odometry CSV has fewer than 2 usable rows: " + path);

    return samples_.size();
}

void OdomTrajectory::push(const nav_msgs::OdometryConstPtr& msg) {
    live_ = true;
    Sample s;
    s.t = msg->header.stamp;
    const auto& pose = msg->pose.pose;
    s.p = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
    s.q = Eigen::Quaterniond(pose.orientation.w, pose.orientation.x,
                             pose.orientation.y, pose.orientation.z).normalized();

    // Out-of-order arrivals are rare but cheap to handle correctly.
    if (!samples_.empty() && s.t < samples_.back().t) {
        auto it = std::upper_bound(samples_.begin(), samples_.end(), s.t,
            [](const ros::Time& t, const Sample& x) { return t < x.t; });
        samples_.insert(it, s);
    } else {
        samples_.push_back(s);
    }

    while (samples_.size() > 1 &&
           (samples_.back().t - samples_.front().t).toSec() > history_sec_)
        samples_.erase(samples_.begin());
}

bool OdomTrajectory::interpolate(const ros::Time& t, Eigen::Matrix4d& T_out) const {
    if (samples_.size() < 2) return false;
    if (t < samples_.front().t || t > samples_.back().t) return false;

    auto it = std::lower_bound(samples_.begin(), samples_.end(), t,
        [](const Sample& s, const ros::Time& target) { return s.t < target; });

    if (it == samples_.begin()) {
        T_out = makeTransform(it->q, it->p);
        return true;
    }

    const Sample& hi = *it;
    const Sample& lo = *std::prev(it);

    const double dt = (hi.t - lo.t).toSec();
    const double a  = (dt > 1e-9) ? (t - lo.t).toSec() / dt : 0.0;

    T_out = makeTransform(lo.q.slerp(a, hi.q), lo.p + a * (hi.p - lo.p));
    return true;
}

bool OdomTrajectory::nearest(const ros::Time& t, double max_dt, Sample& s_out) const {
    if (samples_.empty()) return false;

    auto it = std::lower_bound(samples_.begin(), samples_.end(), t,
        [](const Sample& s, const ros::Time& target) { return s.t < target; });

    const Sample* best = nullptr;
    if (it != samples_.end()) best = &(*it);
    if (it != samples_.begin()) {
        const Sample& prev = *std::prev(it);
        if (!best || std::fabs((prev.t - t).toSec()) < std::fabs((best->t - t).toSec()))
            best = &prev;
    }
    if (!best || std::fabs((best->t - t).toSec()) > max_dt) return false;

    s_out = *best;
    return true;
}
