// ColoriseMap — colourise a pre-built LIO-SAM map with the camera images from
// the same run.
//
// The map points are static and live in the odometry/world frame, so the only
// thing that varies per frame is where the camera was. That pose comes from the
// LIO-SAM odometry, read from a CSV exported by scripts/export_odom_csv.py
// (preferred, because it lets the whole trajectory be available up front) or
// live from the odometry topic.
//
// The image timestamps do not line up with the odometry samples, so the pose is
// interpolated to each image's own capture time — either straight from the
// odometry (CompMode::ODOM) or by anchoring on the nearest odometry sample and
// rotating with the IMU (CompMode::IMU).

#include "colorise.h"

#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <boost/bind.hpp>
#include <csignal>
#include <iomanip>

class MapColouriser;
static MapColouriser* g_instance = nullptr;
static void signalHandler(int);

class MapColouriser {
public:
    MapColouriser(ros::NodeHandle& nh, const CommonParams& params,
                  const std::string& map_frame)
        : nh_(nh), p_(params), map_frame_(map_frame) {
        loadMap();
        loadOdometry();

        img_buffers_.resize(p_.cameras.size());
        for (size_t i = 0; i < p_.cameras.size(); ++i) {
            sub_imgs_.push_back(nh_.subscribe<sensor_msgs::CompressedImage>(
                p_.cameras[i].image_topic, 20,
                boost::bind(&MapColouriser::imageCallback, this, _1, i)));
            ROS_INFO("[ColoriseMap] camera '%s': %s  (%dx%d, %s, margin=%.0f px, "
                     "max view angle=%.0f deg)",
                     p_.cameras[i].name.c_str(), p_.cameras[i].image_topic.c_str(),
                     p_.cameras[i].width, p_.cameras[i].height,
                     p_.cameras[i].distortion_model.c_str(),
                     p_.cameras[i].edge_margin_px, p_.cameras[i].max_view_angle_deg);
        }

        if (p_.compensation_mode == CompMode::IMU)
            sub_imu_ = nh_.subscribe(p_.imu_topic, 500, &MapColouriser::imuCallback, this);
        if (odom_.empty())
            sub_odom_ = nh_.subscribe(p_.odom_topic, 200, &MapColouriser::odomCallback, this);

        pub_coloured_ = nh_.advertise<sensor_msgs::PointCloud2>(p_.output_topic, 1, true);
        pub_raw_map_  = nh_.advertise<sensor_msgs::PointCloud2>("raw_map", 1, true);
        pub_pose_     = nh_.advertise<geometry_msgs::PoseStamped>("camera_pose", 1);
        pub_frustum_  = nh_.advertise<visualization_msgs::MarkerArray>("frustum", 1);

        publishRawMap();

        g_instance = this;
        std::signal(SIGINT, signalHandler);

        timer_ = nh_.createTimer(ros::Duration(0.05), &MapColouriser::onTimer, this);

        ROS_INFO("[ColoriseMap] compensation mode: %s",
                 compModeName(p_.compensation_mode).c_str());
        ROS_INFO("[ColoriseMap] ready — raw map published on ~raw_map, waiting for images");
    }

    void saveFinalMap() {
        if (saved_) return;
        saved_ = true;

        auto out = buildColouredCloud(/*publish_only=*/false);
        if (out->points.empty()) {
            ROS_WARN("[ColoriseMap] no points reached min_color_frames=%d — nothing saved",
                     p_.min_color_frames);
            return;
        }
        if (p_.save_pcd_path.empty()) {
            ROS_WARN("[ColoriseMap] save_pcd_path is not set — nothing saved");
            return;
        }
        ROS_INFO("[ColoriseMap] saving %zu coloured points to %s",
                 out->points.size(), p_.save_pcd_path.c_str());
        if (pcl::io::savePCDFileBinary(p_.save_pcd_path, *out) == 0)
            ROS_INFO("[ColoriseMap] saved.");
        else
            ROS_ERROR("[ColoriseMap] failed to write %s", p_.save_pcd_path.c_str());
    }

private:
    // ── Setup ────────────────────────────────────────────────────────────────

    void loadMap() {
        if (p_.map_pcd_path.empty())
            throw std::runtime_error("map_pcd_path is not set in config.yaml");

        // LIO-SAM writes GlobalMap.pcd as plain XYZI.
        pcl::PointCloud<pcl::PointXYZI>::Ptr raw(new pcl::PointCloud<pcl::PointXYZI>);
        if (pcl::io::loadPCDFile<pcl::PointXYZI>(p_.map_pcd_path, *raw) == -1)
            throw std::runtime_error("Failed to load map PCD: " + p_.map_pcd_path);
        if (raw->empty())
            throw std::runtime_error("Map PCD is empty: " + p_.map_pcd_path);

        map_xyz_.reset(new pcl::PointCloud<pcl::PointXYZ>);
        map_xyz_->points.resize(raw->points.size());
        map_intensity_.resize(raw->points.size());
        P3_.reserve(raw->points.size());

        for (size_t i = 0; i < raw->points.size(); ++i) {
            const auto& p = raw->points[i];
            map_xyz_->points[i].x = p.x;
            map_xyz_->points[i].y = p.y;
            map_xyz_->points[i].z = p.z;
            map_intensity_[i]     = p.intensity;
            P3_.emplace_back(p.x, p.y, p.z);
        }
        map_xyz_->width  = map_xyz_->points.size();
        map_xyz_->height = 1;

        colour_sum_.assign(raw->points.size(), Eigen::Vector3i::Zero());
        colour_cnt_.assign(raw->points.size(), 0);

        ROS_INFO("[ColoriseMap] loaded %zu map points from %s",
                 raw->points.size(), p_.map_pcd_path.c_str());

        ROS_INFO("[ColoriseMap] building kd-tree for range culling...");
        kdtree_.setInputCloud(map_xyz_);
        ROS_INFO("[ColoriseMap] kd-tree ready (cull radius %.1f m)", p_.map_max_range);
    }

    void loadOdometry() {
        if (p_.odom_csv_path.empty()) {
            ROS_WARN("[ColoriseMap] odom_csv_path not set — falling back to the live "
                     "topic %s. Exporting a CSV with scripts/export_odom_csv.py is "
                     "preferred: the full trajectory is then known up front.",
                     p_.odom_topic.c_str());
            return;
        }
        const size_t n = odom_.loadCsv(p_.odom_csv_path);
        ROS_INFO("[ColoriseMap] loaded %zu odometry samples from %s  (t = %.3f .. %.3f)",
                 n, p_.odom_csv_path.c_str(), odom_.front().toSec(), odom_.back().toSec());
    }

    // ── Callbacks ────────────────────────────────────────────────────────────

    void imageCallback(const sensor_msgs::CompressedImageConstPtr& msg, size_t idx) {
        img_buffers_[idx].push_back(msg);
        cleanOldMsgs(img_buffers_[idx], msg->header.stamp, 5.0);
    }

    void imuCallback(const sensor_msgs::ImuConstPtr& msg) { imu_.push(msg); }

    void odomCallback(const nav_msgs::OdometryConstPtr& msg) { odom_.push(msg); }

    void onTimer(const ros::TimerEvent&) {
        for (const auto& b : img_buffers_)
            if (b.empty()) {
                ROS_WARN_THROTTLE(5.0, "[ColoriseMap] waiting for images on all %zu cameras",
                                  p_.cameras.size());
                return;
            }
        if (odom_.empty()) {
            ROS_WARN_THROTTLE(5.0, "[ColoriseMap] no odometry yet (topic %s)",
                              p_.odom_topic.c_str());
            return;
        }

        // Drive off the first camera; match the rest to it.
        const ros::Time t_ref = img_buffers_[0].front()->header.stamp;

        std::vector<sensor_msgs::CompressedImageConstPtr> imgs(p_.cameras.size());
        imgs[0] = img_buffers_[0].front();
        for (size_t i = 1; i < p_.cameras.size(); ++i) {
            imgs[i] = findClosest(img_buffers_[i], t_ref, p_.max_time_offset);
            if (!imgs[i]) {
                // The partner camera may simply not have arrived yet; only drop
                // the frame once its buffer has clearly moved past t_ref.
                if (img_buffers_[i].back()->header.stamp > t_ref + ros::Duration(p_.max_time_offset)) {
                    ROS_WARN_THROTTLE(5.0, "[ColoriseMap] no match on '%s' for t=%.3f — skipping frame",
                                      p_.cameras[i].name.c_str(), t_ref.toSec());
                    img_buffers_[0].pop_front();
                }
                return;
            }
        }

        img_buffers_[0].pop_front();
        processFrame(imgs);
    }

    // ── Pose at an image timestamp ───────────────────────────────────────────
    //
    // Returns T_world_lidar at t. LIO-SAM's mapping odometry is the pose of the
    // lidar frame (params.yaml sets lidarFrame: os_sensor), so composing with
    // the calibration extrinsic gives the camera pose directly — no TF needed.

    bool poseAt(const ros::Time& t, Eigen::Matrix4d& T_world_lidar) {
        switch (p_.compensation_mode) {
            case CompMode::NONE: {
                // Nearest odometry sample, no interpolation — the baseline that
                // shows what the timestamp offset actually costs.
                OdomTrajectory::Sample s;
                if (!odom_.nearest(t, 0.5, s)) return false;
                T_world_lidar = makeTransform(s.q, s.p);
                return true;
            }
            case CompMode::ODOM: {
                return odom_.interpolate(t, T_world_lidar);
            }
            case CompMode::IMU: {
                // Anchor position on the nearest odometry sample, then carry the
                // orientation to t with the IMU. Position is held: an
                // orientation-only AHRS gives no velocity to extrapolate with,
                // and over the tens of milliseconds involved the rotation is the
                // dominant term. Use CompMode::ODOM when translation matters.
                OdomTrajectory::Sample s;
                if (!odom_.nearest(t, 0.5, s)) return false;

                Eigen::Quaterniond q_anchor, q_target;
                if (!imu_.interpolate(s.t, q_anchor) || !imu_.interpolate(t, q_target)) {
                    ROS_WARN_THROTTLE(5.0,
                        "[ColoriseMap] IMU does not cover t=%.3f (buffer=%zu) — "
                        "using the nearest odometry sample instead", t.toSec(), imu_.size());
                    T_world_lidar = makeTransform(s.q, s.p);
                    return true;
                }
                // R_world_lidar(t) = R_world_lidar(anchor) * delta, with delta
                // expressed in the lidar frame via the IMU-to-lidar extrinsic.
                const Eigen::Matrix3d R_delta =
                    p_.R_imu_lidar.transpose() *
                    (q_anchor.toRotationMatrix().transpose() * q_target.toRotationMatrix()) *
                    p_.R_imu_lidar;
                T_world_lidar = makeTransform(
                    Eigen::Quaterniond(s.q.toRotationMatrix() * R_delta), s.p);
                return true;
            }
        }
        return false;
    }

    // ── Per-frame processing ─────────────────────────────────────────────────

    void processFrame(const std::vector<sensor_msgs::CompressedImageConstPtr>& imgs) {
        size_t coloured_this_frame = 0;

        for (size_t i = 0; i < p_.cameras.size(); ++i) {
            const CameraCalib& cam = p_.cameras[i];
            const ros::Time    t   = imgs[i]->header.stamp;

            Eigen::Matrix4d T_world_lidar;
            if (!poseAt(t, T_world_lidar)) {
                ROS_WARN_THROTTLE(2.0,
                    "[ColoriseMap] no pose for t=%.3f (odometry covers %.3f .. %.3f)",
                    t.toSec(), odom_.front().toSec(), odom_.back().toSec());
                continue;
            }

            const Eigen::Matrix4d T_world_cam      = T_world_lidar * cam.T_lidar_cam;
            const Eigen::Matrix4d T_cam_from_world = T_world_cam.inverse();
            const Eigen::Vector3d cam_pos          = T_world_cam.block<3, 1>(0, 3);

            // Range cull: only map points near the camera can be in view.
            // Without this every frame would project all ~2.3 M map points.
            pcl::PointXYZ query;
            query.x = static_cast<float>(cam_pos.x());
            query.y = static_cast<float>(cam_pos.y());
            query.z = static_cast<float>(cam_pos.z());

            std::vector<int>   candidates;
            std::vector<float> sqr_dists;
            kdtree_.radiusSearch(query, p_.map_max_range, candidates, sqr_dists);
            if (candidates.empty()) {
                ROS_WARN_THROTTLE(2.0,
                    "[ColoriseMap] no map points within %.1f m of the '%s' camera at "
                    "(%.1f, %.1f, %.1f) — is the map from this run?",
                    p_.map_max_range, cam.name.c_str(),
                    cam_pos.x(), cam_pos.y(), cam_pos.z());
                continue;
            }

            cv::Mat img = cv::imdecode(cv::Mat(imgs[i]->data), cv::IMREAD_COLOR);
            if (img.empty()) {
                ROS_WARN_THROTTLE(2.0, "[ColoriseMap] failed to decode image from '%s'",
                                  cam.name.c_str());
                continue;
            }

            projectAndSample(P3_, candidates, img, cam, T_cam_from_world,
                             p_.occlusion_check, p_.occlusion_cell_px,
                             p_.occlusion_depth_tol,
                             [&](int idx, std::uint32_t rgb) {
                                 colour_sum_[idx] += Eigen::Vector3i((rgb >> 16) & 0xFF,
                                                                     (rgb >>  8) & 0xFF,
                                                                     (rgb      ) & 0xFF);
                                 colour_cnt_[idx] += 1;
                                 ++coloured_this_frame;
                             });

            publishPose(T_world_cam, t);
            publishFrustum(T_world_cam, cam, t, static_cast<int>(i));
        }

        ++frames_processed_;
        if (coloured_this_frame == 0) {
            ROS_WARN_THROTTLE(3.0,
                "[ColoriseMap] frame %zu coloured 0 points — check that the map, the "
                "odometry CSV and the bag all come from the same run",
                frames_processed_);
        }

        if (frames_processed_ % 10 == 0) publishColoured();

        if (frames_processed_ % 20 == 0) {
            size_t done = 0;
            for (int c : colour_cnt_) if (c >= p_.min_color_frames) ++done;
            ROS_INFO("[ColoriseMap] frame %zu: %zu/%zu map points coloured (%.1f%%)",
                     frames_processed_, done, colour_cnt_.size(),
                     100.0 * done / colour_cnt_.size());
        }
    }

    // ── Output ───────────────────────────────────────────────────────────────

    pcl::PointCloud<PointXYZRGBIntensity>::Ptr buildColouredCloud(bool publish_only) {
        auto out = boost::make_shared<pcl::PointCloud<PointXYZRGBIntensity>>();
        out->header.frame_id = map_frame_;
        out->is_dense = false;
        out->height   = 1;

        for (size_t i = 0; i < colour_cnt_.size(); ++i) {
            const bool has_colour = colour_cnt_[i] >= p_.min_color_frames;
            if (!has_colour && !p_.keep_uncolored_points) continue;

            PointXYZRGBIntensity pt;
            pt.x         = map_xyz_->points[i].x;
            pt.y         = map_xyz_->points[i].y;
            pt.z         = map_xyz_->points[i].z;
            pt.intensity = map_intensity_[i];
            pt.t = 0; pt.reflectivity = 0; pt.ring = 0; pt.ambient = 0; pt.range = 0;

            if (has_colour) {
                const Eigen::Vector3i c = colour_sum_[i] / colour_cnt_[i];
                pt.rgb = (static_cast<std::uint32_t>(c.x()) << 16) |
                         (static_cast<std::uint32_t>(c.y()) <<  8) |
                         (static_cast<std::uint32_t>(c.z()));
            } else {
                pt.rgb = 0;
            }
            out->points.push_back(pt);
        }
        out->width = out->points.size();
        (void)publish_only;
        return out;
    }

    void publishColoured() {
        auto cloud = buildColouredCloud(true);
        if (cloud->points.empty()) return;

        sensor_msgs::PointCloud2 msg;
        pcl::PCLPointCloud2 pcl_pc2;
        pcl::toPCLPointCloud2(*cloud, pcl_pc2);
        pcl_conversions::fromPCL(pcl_pc2, msg);
        msg.header.stamp    = ros::Time::now();
        msg.header.frame_id = map_frame_;
        pub_coloured_.publish(msg);
    }

    void publishRawMap() {
        pcl::PointCloud<PointXYZRGBIntensity> grey;
        grey.points.resize(map_xyz_->points.size());
        for (size_t i = 0; i < map_xyz_->points.size(); ++i) {
            auto& pt = grey.points[i];
            pt.x = map_xyz_->points[i].x;
            pt.y = map_xyz_->points[i].y;
            pt.z = map_xyz_->points[i].z;
            pt.intensity = map_intensity_[i];
            pt.t = 0; pt.reflectivity = 0; pt.ring = 0; pt.ambient = 0; pt.range = 0;
            const auto g = static_cast<std::uint32_t>(
                std::min(255.0f, map_intensity_[i] * 0.5f + 80.0f));
            pt.rgb = (g << 16) | (g << 8) | g;
        }
        grey.width    = grey.points.size();
        grey.height   = 1;
        grey.is_dense = false;

        sensor_msgs::PointCloud2 msg;
        pcl::PCLPointCloud2 pcl_pc2;
        pcl::toPCLPointCloud2(grey, pcl_pc2);
        pcl_conversions::fromPCL(pcl_pc2, msg);
        msg.header.stamp    = ros::Time::now();
        msg.header.frame_id = map_frame_;
        pub_raw_map_.publish(msg);
        ROS_INFO("[ColoriseMap] published raw map (%zu points, frame '%s')",
                 grey.points.size(), map_frame_.c_str());
    }

    void publishPose(const Eigen::Matrix4d& T, const ros::Time& t) {
        geometry_msgs::PoseStamped msg;
        msg.header.stamp    = t;
        msg.header.frame_id = map_frame_;
        const Eigen::Vector3d    p(T.block<3, 1>(0, 3));
        Eigen::Quaterniond q(Eigen::Matrix3d(T.block<3, 3>(0, 0)));
        q.normalize();
        msg.pose.position.x = p.x();
        msg.pose.position.y = p.y();
        msg.pose.position.z = p.z();
        msg.pose.orientation.w = q.w();
        msg.pose.orientation.x = q.x();
        msg.pose.orientation.y = q.y();
        msg.pose.orientation.z = q.z();
        pub_pose_.publish(msg);
    }

    // Draws the camera's view cone in the map frame. If this does not sit where
    // the robot actually was, the extrinsic or the odometry is wrong — it is the
    // quickest way to spot a bad calibration key.
    void publishFrustum(const Eigen::Matrix4d& T_world_cam, const CameraCalib& cam,
                        const ros::Time& t, int id) {
        const double fx = cam.K.at<double>(0, 0), fy = cam.K.at<double>(1, 1);
        const double cx = cam.K.at<double>(0, 2), cy = cam.K.at<double>(1, 2);
        const double depth = 3.0;
        const double m = cam.edge_margin_px;

        auto unproject = [&](double u, double v) {
            return Eigen::Vector4d((u - cx) / fx * depth, (v - cy) / fy * depth, depth, 1.0);
        };
        const std::vector<Eigen::Vector4d> corners = {
            unproject(m, m), unproject(cam.width - m, m),
            unproject(cam.width - m, cam.height - m), unproject(m, cam.height - m)};

        auto toPoint = [&](const Eigen::Vector4d& pc) {
            const Eigen::Vector4d pw = T_world_cam * pc;
            geometry_msgs::Point q;
            q.x = pw.x(); q.y = pw.y(); q.z = pw.z();
            return q;
        };

        visualization_msgs::Marker marker;
        marker.header.frame_id = map_frame_;
        marker.header.stamp    = t;
        marker.ns              = "frustum";
        marker.id              = id;
        marker.type            = visualization_msgs::Marker::LINE_LIST;
        marker.action          = visualization_msgs::Marker::ADD;
        marker.scale.x         = 0.03;
        marker.color.r = (id == 0) ? 0.2f : 0.9f;
        marker.color.g = 0.8f;
        marker.color.b = (id == 0) ? 0.2f : 0.9f;
        marker.color.a = 0.8f;
        marker.lifetime = ros::Duration(1.0);

        const geometry_msgs::Point origin = toPoint(Eigen::Vector4d(0, 0, 0, 1));
        for (const auto& c : corners) {
            marker.points.push_back(origin);
            marker.points.push_back(toPoint(c));
        }
        for (int i = 0; i < 4; ++i) {
            marker.points.push_back(toPoint(corners[i]));
            marker.points.push_back(toPoint(corners[(i + 1) % 4]));
        }

        visualization_msgs::MarkerArray arr;
        arr.markers.push_back(marker);
        pub_frustum_.publish(arr);
    }

    ros::NodeHandle nh_;
    CommonParams    p_;
    std::string     map_frame_;

    ros::Subscriber              sub_imu_, sub_odom_;
    std::vector<ros::Subscriber> sub_imgs_;
    ros::Publisher  pub_coloured_, pub_raw_map_, pub_pose_, pub_frustum_;
    ros::Timer      timer_;

    std::vector<std::deque<sensor_msgs::CompressedImageConstPtr>> img_buffers_;

    ImuBuffer      imu_;
    OdomTrajectory odom_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr map_xyz_;
    std::vector<float>                  map_intensity_;
    std::vector<cv::Point3f>            P3_;
    pcl::KdTreeFLANN<pcl::PointXYZ>     kdtree_;

    std::vector<Eigen::Vector3i> colour_sum_;
    std::vector<int>             colour_cnt_;

    size_t frames_processed_ = 0;
    bool   saved_            = false;
};

static void signalHandler(int) {
    if (g_instance) g_instance->saveFinalMap();
    ros::shutdown();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "colorise_map_node", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh("~");

    std::string default_config =
        ros::package::getPath("point_cloud_projection") + "/configs/config.yaml";
    std::string config_path;
    nh.param("config_path", config_path, default_config);

    std::string map_frame;
    nh.param("map_frame", map_frame, std::string("odom"));

    CommonParams params;
    try {
        loadCommonParams(config_path, params);
    } catch (const std::exception& e) {
        ROS_FATAL("[ColoriseMap] configuration error: %s", e.what());
        return 1;
    }

    ROS_INFO("[ColoriseMap] waiting %.2f s for sensor startup...",
             params.initial_startup_delay);
    ros::Duration(params.initial_startup_delay).sleep();

    try {
        MapColouriser node(nh, params, map_frame);
        ros::spin();
    } catch (const std::exception& e) {
        ROS_FATAL("[ColoriseMap] startup failed: %s", e.what());
        return 1;
    }
    return 0;
}
