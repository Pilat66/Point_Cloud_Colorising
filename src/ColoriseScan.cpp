// ColoriseScan — colourise each instantaneous lidar sweep with the camera
// images captured closest to it.
//
// The cameras are not hardware-synchronised with the lidar (in the reference
// bag the images are consistently ~22 ms older than the sweep they match), so
// the static extrinsic alone smears colour across edges whenever the platform
// moves. The pose delta over that offset is estimated either from the IMU
// (rotation only) or from an odometry trajectory (full 6-DoF) — see CompMode.

#include "colorise.h"

#include <boost/bind.hpp>
#include <cstring>

class ScanColouriser {
public:
    explicit ScanColouriser(ros::NodeHandle& nh, const CommonParams& params)
        : nh_(nh), p_(params) {
        img_buffers_.resize(p_.cameras.size());

        sub_cloud_ = nh_.subscribe(p_.pointcloud_topic, 10,
                                   &ScanColouriser::cloudCallback, this);

        for (size_t i = 0; i < p_.cameras.size(); ++i) {
            sub_imgs_.push_back(nh_.subscribe<sensor_msgs::CompressedImage>(
                p_.cameras[i].image_topic, 10,
                boost::bind(&ScanColouriser::imageCallback, this, _1, i)));
            ROS_INFO("[ColoriseScan] camera '%s': %s  (%dx%d, %s, margin=%.0f px, "
                     "max view angle=%.0f deg, min dist=%.1f m)",
                     p_.cameras[i].name.c_str(), p_.cameras[i].image_topic.c_str(),
                     p_.cameras[i].width, p_.cameras[i].height,
                     p_.cameras[i].distortion_model.c_str(),
                     p_.cameras[i].edge_margin_px, p_.cameras[i].max_view_angle_deg,
                     p_.cameras[i].min_camera_dist);
        }

        if (p_.compensation_mode == CompMode::IMU)
            sub_imu_ = nh_.subscribe(p_.imu_topic, 500,
                                     &ScanColouriser::imuCallback, this);
        if (p_.compensation_mode == CompMode::ODOM)
            sub_odom_ = nh_.subscribe(p_.odom_topic, 200,
                                      &ScanColouriser::odomCallback, this);

        pub_ = nh_.advertise<sensor_msgs::PointCloud2>(p_.output_topic, 1);

        ROS_INFO("[ColoriseScan] compensation mode: %s",
                 compModeName(p_.compensation_mode).c_str());
        ROS_INFO("[ColoriseScan] publishing to %s", p_.output_topic.c_str());
    }

private:
    // ── Callbacks ────────────────────────────────────────────────────────────

    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
        cloud_buffer_.push_back(msg);
        cleanOldMsgs(cloud_buffer_, msg->header.stamp);
        trySyncAndProcess();
    }

    void imageCallback(const sensor_msgs::CompressedImageConstPtr& msg, size_t idx) {
        img_buffers_[idx].push_back(msg);
        cleanOldMsgs(img_buffers_[idx], msg->header.stamp);
    }

    void imuCallback(const sensor_msgs::ImuConstPtr& msg) { imu_.push(msg); }

    void odomCallback(const nav_msgs::OdometryConstPtr& msg) { odom_.push(msg); }

    // ── Sync ─────────────────────────────────────────────────────────────────

    void trySyncAndProcess() {
        for (const auto& b : img_buffers_)
            if (b.empty()) return;
        if (cloud_buffer_.empty()) return;

        for (auto it = cloud_buffer_.begin(); it != cloud_buffer_.end(); ++it) {
            const ros::Time t_lidar = (*it)->header.stamp;

            // The pose source has to already cover this sweep, otherwise
            // interpolation fails and the mode silently degrades to the static
            // extrinsic. Odometry in particular can lag the lidar (a SLAM node has to
            // process the sweep first), so hold the sweep until it catches up.
            if (!poseSourceReady(t_lidar)) return;

            std::vector<sensor_msgs::CompressedImageConstPtr> imgs(p_.cameras.size());
            bool complete = true;
            for (size_t i = 0; i < p_.cameras.size(); ++i) {
                imgs[i] = findClosest(img_buffers_[i], t_lidar, p_.max_time_offset);
                if (!imgs[i]) { complete = false; break; }
            }
            if (!complete) continue;

            process(*it, imgs, t_lidar);
            cloud_buffer_.erase(cloud_buffer_.begin(), it + 1);
            return;
        }

        ROS_WARN_THROTTLE(5.0,
            "[ColoriseScan] no image match within max_time_offset=%.3f s for any "
            "buffered sweep — raise max_time_offset if this persists",
            p_.max_time_offset);
    }

    // True once the configured pose source spans t (plus the pairing window, so
    // the camera timestamp on either side of the sweep is covered too).
    bool poseSourceReady(const ros::Time& t) const {
        const ros::Duration pad(p_.max_time_offset);
        switch (p_.compensation_mode) {
            case CompMode::NONE:
                return true;
            case CompMode::IMU:
                if (imu_.covers(t + pad)) return true;
                ROS_WARN_THROTTLE(5.0, "[ColoriseScan] waiting for IMU to cover t=%.3f "
                                  "(buffer=%zu)", t.toSec(), imu_.size());
                return false;
            case CompMode::ODOM:
                if (odom_.size() >= 2 && odom_.back() >= t + pad &&
                    odom_.front() <= t - pad) return true;
                ROS_WARN_THROTTLE(5.0, "[ColoriseScan] waiting for odometry to cover "
                                  "t=%.3f (samples=%zu, span %.3f..%.3f, need %.3f..%.3f, "
                                  "topic %s)",
                                  t.toSec(), odom_.size(),
                                  odom_.size() ? odom_.front().toSec() : 0.0,
                                  odom_.size() ? odom_.back().toSec()  : 0.0,
                                  (t - pad).toSec(), (t + pad).toSec(),
                                  p_.odom_topic.c_str());
                return false;
        }
        return true;
    }

    // ── Motion compensation ──────────────────────────────────────────────────
    //
    // Returns the transform taking points from the lidar frame at t_lidar into
    // the camera frame at t_cam:
    //
    //   T_cam_from_lidar = T_cam_lidar(static) * T_lidar(t_cam)_from_lidar(t_lidar)
    //
    // The delta term is identity under CompMode::NONE, rotation-only under IMU,
    // and full 6-DoF under ODOM. On missing data it degrades to the static
    // extrinsic and warns rather than dropping the sweep.

    bool computeCamFromLidar(const CameraCalib& cam,
                             const ros::Time& t_lidar,
                             const ros::Time& t_cam,
                             Eigen::Matrix4d& T_out) {
        Eigen::Matrix4d delta = Eigen::Matrix4d::Identity();
        const double dt = (t_cam - t_lidar).toSec();

        if (p_.compensation_mode != CompMode::NONE && std::fabs(dt) > 1e-4) {
            if (p_.compensation_mode == CompMode::IMU) {
                Eigen::Quaterniond q_lidar, q_cam;
                if (imu_.interpolate(t_lidar, q_lidar) && imu_.interpolate(t_cam, q_cam)) {
                    // R_world_lidar(t) = R_world_imu(t) * R_imu_lidar
                    const Eigen::Matrix3d R_delta =
                        p_.R_imu_lidar.transpose() *
                        (q_cam.toRotationMatrix().transpose() * q_lidar.toRotationMatrix()) *
                        p_.R_imu_lidar;
                    delta.block<3, 3>(0, 0) = R_delta;
                } else {
                    ROS_WARN_THROTTLE(5.0,
                        "[ColoriseScan] IMU does not cover dt=%.4f s (buffer=%zu) — "
                        "falling back to the static extrinsic", dt, imu_.size());
                }
            } else {  // ODOM
                Eigen::Matrix4d T_at_lidar, T_at_cam;
                if (odom_.interpolate(t_lidar, T_at_lidar) &&
                    odom_.interpolate(t_cam,   T_at_cam)) {
                    delta = T_at_cam.inverse() * T_at_lidar;
                } else {
                    ROS_WARN_THROTTLE(5.0,
                        "[ColoriseScan] odometry does not cover dt=%.4f s (samples=%zu) — "
                        "falling back to the static extrinsic", dt, odom_.size());
                }
            }
        }

        T_out = cam.T_cam_lidar * delta;

        // Report how much work the compensation is actually doing. If this is
        // ~0 the mode is having no effect and something upstream is wrong.
        const double rot_deg = Eigen::AngleAxisd(
            Eigen::Matrix3d(delta.block<3, 3>(0, 0))).angle() * 180.0 / M_PI;
        const double trans_mm = delta.block<3, 1>(0, 3).norm() * 1e3;
        ROS_INFO_THROTTLE(5.0,
            "[ColoriseScan] compensation [%s] dt=%+.1f ms -> rot=%.3f deg, trans=%.1f mm",
            cam.name.c_str(), dt * 1e3, rot_deg, trans_mm);

        return true;
    }

    // ── Processing ───────────────────────────────────────────────────────────

    void process(const sensor_msgs::PointCloud2ConstPtr& cloud_msg,
                 const std::vector<sensor_msgs::CompressedImageConstPtr>& imgs,
                 const ros::Time& t_lidar) {
        pcl::PointCloud<PointXYZRGBIntensity>::Ptr in(
            new pcl::PointCloud<PointXYZRGBIntensity>);
        if (!unpackCloud(cloud_msg, in)) return;

        std::vector<cv::Point3f> P3;
        P3.reserve(in->points.size());
        for (const auto& pt : in->points)
            P3.emplace_back(pt.x, pt.y, pt.z);

        // Height gate in the lidar frame, applied once for all cameras.
        std::vector<int> candidates;
        candidates.reserve(P3.size());
        for (size_t i = 0; i < P3.size(); ++i)
            if (P3[i].z <= p_.max_lidar_z) candidates.push_back(static_cast<int>(i));

        if (candidates.empty()) {
            ROS_WARN_THROTTLE(5.0, "[ColoriseScan] max_lidar_z=%.1f rejected every point "
                              "in this sweep", p_.max_lidar_z);
            return;
        }

        // Accumulate colour per point so a point seen by several cameras gets
        // the average rather than whichever camera happened to run last.
        std::vector<Eigen::Vector3i> sum(P3.size(), Eigen::Vector3i::Zero());
        std::vector<int>             cnt(P3.size(), 0);

        for (size_t i = 0; i < p_.cameras.size(); ++i) {
            const CameraCalib& cam = p_.cameras[i];

            cv::Mat img = cv::imdecode(cv::Mat(imgs[i]->data), cv::IMREAD_COLOR);
            if (img.empty()) {
                ROS_WARN_THROTTLE(2.0, "[ColoriseScan] failed to decode image from '%s'",
                                  cam.name.c_str());
                continue;
            }

            Eigen::Matrix4d T_cam_from_lidar;
            if (!computeCamFromLidar(cam, t_lidar, imgs[i]->header.stamp, T_cam_from_lidar))
                continue;

            projectAndSample(P3, candidates, img, cam, T_cam_from_lidar,
                             p_.occlusion_check, p_.occlusion_cell_px,
                             p_.occlusion_depth_tol,
                             p_.occlusion_max_depth, p_.occlusion_ray_margin,
                             [&](int idx, std::uint32_t rgb) {
                                 sum[idx] += Eigen::Vector3i((rgb >> 16) & 0xFF,
                                                             (rgb >>  8) & 0xFF,
                                                             (rgb      ) & 0xFF);
                                 cnt[idx] += 1;
                             });
        }

        // Build the output cloud.
        auto out = boost::make_shared<pcl::PointCloud<PointXYZRGBIntensity>>();
        out->header.frame_id = cloud_msg->header.frame_id;
        out->is_dense = false;
        out->height   = 1;
        out->points.reserve(p_.keep_uncolored_points ? in->points.size() : P3.size() / 4);

        size_t coloured = 0;
        for (size_t i = 0; i < in->points.size(); ++i) {
            const bool has_colour = cnt[i] > 0;
            if (!has_colour && !p_.keep_uncolored_points) continue;

            PointXYZRGBIntensity pt = in->points[i];
            if (has_colour) {
                const Eigen::Vector3i c = sum[i] / cnt[i];
                pt.rgb = (static_cast<std::uint32_t>(c.x()) << 16) |
                         (static_cast<std::uint32_t>(c.y()) <<  8) |
                         (static_cast<std::uint32_t>(c.z()));
                ++coloured;
            } else {
                pt.rgb = 0;
            }
            out->points.push_back(pt);
        }
        out->width = out->points.size();

        ROS_INFO_THROTTLE(2.0, "[ColoriseScan] t=%.3f  %zu/%zu points coloured (%.1f%%)",
                          t_lidar.toSec(), coloured, in->points.size(),
                          in->points.empty() ? 0.0 : 100.0 * coloured / in->points.size());

        pcl::PointCloud<PointXYZRGBIntensity>::Ptr cleaned(
            new pcl::PointCloud<PointXYZRGBIntensity>);
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*out, *cleaned, indices);

        sensor_msgs::PointCloud2 out_msg;
        pcl::PCLPointCloud2 pcl_pc2;
        pcl::toPCLPointCloud2(*cleaned, pcl_pc2);
        pcl_conversions::fromPCL(pcl_pc2, out_msg);
        out_msg.header = cloud_msg->header;
        pub_.publish(out_msg);
    }

    // Ouster PointCloud2 -> PointXYZRGBIntensity. Field offsets are resolved
    // from the message rather than hardcoded, so a layout change is reported
    // instead of silently producing garbage.
    bool unpackCloud(const sensor_msgs::PointCloud2ConstPtr& msg,
                     pcl::PointCloud<PointXYZRGBIntensity>::Ptr& out) {
        auto offsetOf = [&](const std::string& name) -> int {
            for (const auto& f : msg->fields)
                if (f.name == name) return static_cast<int>(f.offset);
            return -1;
        };

        const int o_x = offsetOf("x"), o_y = offsetOf("y"), o_z = offsetOf("z");
        if (o_x < 0 || o_y < 0 || o_z < 0) {
            ROS_ERROR_THROTTLE(5.0, "[ColoriseScan] cloud on %s has no x/y/z fields",
                               p_.pointcloud_topic.c_str());
            return false;
        }
        const int o_i = offsetOf("intensity"), o_t = offsetOf("t");
        const int o_r = offsetOf("reflectivity"), o_ring = offsetOf("ring");
        const int o_a = offsetOf("ambient"), o_range = offsetOf("range");

        const size_t step = msg->point_step;
        const size_t n    = static_cast<size_t>(msg->width) * msg->height;

        out->header.frame_id = msg->header.frame_id;
        out->is_dense = false;
        out->height   = 1;
        out->points.resize(n);

        for (size_t i = 0; i < n; ++i) {
            const uint8_t* ptr = &msg->data[i * step];
            PointXYZRGBIntensity& pt = out->points[i];
            std::memcpy(&pt.x, ptr + o_x, sizeof(float));
            std::memcpy(&pt.y, ptr + o_y, sizeof(float));
            std::memcpy(&pt.z, ptr + o_z, sizeof(float));
            pt.intensity    = (o_i     >= 0) ? *reinterpret_cast<const float*>(ptr + o_i) : 0.f;
            pt.t            = (o_t     >= 0) ? *reinterpret_cast<const std::uint32_t*>(ptr + o_t) : 0;
            pt.reflectivity = (o_r     >= 0) ? *reinterpret_cast<const std::uint16_t*>(ptr + o_r) : 0;
            pt.ring         = (o_ring  >= 0) ? *reinterpret_cast<const std::uint16_t*>(ptr + o_ring) : 0;
            pt.ambient      = (o_a     >= 0) ? *reinterpret_cast<const std::uint16_t*>(ptr + o_a) : 0;
            pt.range        = (o_range >= 0) ? *reinterpret_cast<const std::uint32_t*>(ptr + o_range) : 0;
            pt.rgb          = 0;
        }
        out->width = n;
        return true;
    }

    ros::NodeHandle nh_;
    CommonParams    p_;

    ros::Subscriber              sub_cloud_, sub_imu_, sub_odom_;
    std::vector<ros::Subscriber> sub_imgs_;
    ros::Publisher               pub_;

    std::deque<sensor_msgs::PointCloud2ConstPtr>                 cloud_buffer_;
    std::vector<std::deque<sensor_msgs::CompressedImageConstPtr>> img_buffers_;

    ImuBuffer      imu_;
    OdomTrajectory odom_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "colorise_scan_node");
    ros::NodeHandle nh("~");

    std::string default_config =
        ros::package::getPath("point_cloud_projection") + "/configs/config.yaml";
    std::string config_path;
    nh.param("config_path", config_path, default_config);

    CommonParams params;
    try {
        loadCommonParams(config_path, params);

        // Optional ROS-param override, so the mode can be switched from the
        // launch file without editing config.yaml.
        std::string mode_override;
        nh.param("compensation_mode", mode_override, std::string(""));
        if (!mode_override.empty())
            params.compensation_mode = parseCompMode(mode_override);
    } catch (const std::exception& e) {
        ROS_FATAL("[ColoriseScan] configuration error: %s", e.what());
        return 1;
    }

    ROS_INFO("[ColoriseScan] waiting %.2f s for sensor startup...",
             params.initial_startup_delay);
    ros::Duration(params.initial_startup_delay).sleep();

    ScanColouriser node(nh, params);
    ros::spin();
    return 0;
}
