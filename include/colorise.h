#ifndef COLORISE_H
#define COLORISE_H

#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>

#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
// PointXYZRGBIntensity is a custom point type, so the filter templates are not
// pre-instantiated in libpcl_filters — pull in the implementation header.
#include <pcl/filters/impl/filter.hpp>

#include <deque>
#include <functional>
#include <limits>
#include <string>
#include <vector>

#include "calibration.h"

// ─────────────────────────────────────────────────────────────────────────────
// Point type: Ouster layout plus an RGB field.
// ─────────────────────────────────────────────────────────────────────────────
struct PointXYZRGBIntensity {
    PCL_ADD_POINT4D;
    float         intensity;
    std::uint32_t t;
    std::uint16_t reflectivity;
    std::uint16_t ring;
    std::uint16_t ambient;
    std::uint32_t range;
    std::uint32_t rgb;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZRGBIntensity,
                                (float,         x,            x)
                                (float,         y,            y)
                                (float,         z,            z)
                                (float,         intensity,    intensity)
                                (std::uint32_t, t,            t)
                                (std::uint16_t, reflectivity, reflectivity)
                                (std::uint16_t, ring,         ring)
                                (std::uint16_t, ambient,      ambient)
                                (std::uint32_t, range,        range)
                                (std::uint32_t, rgb,          rgb)
)

// ─────────────────────────────────────────────────────────────────────────────
// Timestamp-offset compensation mode.
//
// The camera and lidar are not hardware-synchronised: in the reference bag the
// images are consistently ~22 ms older than the lidar sweep they get matched
// to. Projecting without correcting for that smears colour across edges
// whenever the platform is moving.
//
//   NONE : no compensation, static extrinsic only (baseline / debugging).
//   IMU  : rotation-only correction from the AHRS quaternion (SLERP between
//          IMU samples). Cheap, always available, and captures the dominant
//          error for a platform that is mostly rotating. It cannot recover
//          translation, because orientation-only IMU output carries no velocity.
//   ODOM : full 6-DoF correction from the odometry trajectory (SLERP + linear
//          interpolation of position). More accurate while translating, but
//          only usable where odometry exists.
// ─────────────────────────────────────────────────────────────────────────────
enum class CompMode { NONE, IMU, ODOM };

CompMode parseCompMode(const std::string& s);
std::string compModeName(CompMode m);

// ─────────────────────────────────────────────────────────────────────────────
// Parameters shared by both nodes, loaded from configs/config.yaml.
// ─────────────────────────────────────────────────────────────────────────────
struct CommonParams {
    std::string calibration_path;
    std::vector<CameraCalib> cameras;

    // IMU-frame rotation relative to the lidar, from calibration.yaml.
    Eigen::Matrix3d R_imu_lidar = Eigen::Matrix3d::Identity();

    std::string pointcloud_topic;
    std::string imu_topic;
    std::string odom_topic;
    std::string output_topic;

    double   max_time_offset       = 0.05;
    double   initial_startup_delay = 0.1;
    bool     keep_uncolored_points = false;
    double   max_lidar_z           = 100.0;
    CompMode compensation_mode     = CompMode::IMU;

    // Occlusion rejection (z-buffer). Essential for map colourisation, where a
    // wall and everything behind it project to the same pixels.
    bool   occlusion_check     = true;
    double occlusion_cell_px   = 4.0;
    double occlusion_depth_tol = 0.3;

    // Map node
    std::string map_pcd_path;
    std::string odom_csv_path;
    std::string save_pcd_path;
    int         min_color_frames = 1;
    double      map_max_range    = 30.0;
    // How often the accumulating coloured map is republished. Lower = smoother
    // fill-in in RViz, at the cost of resending the whole cloud more often.
    int         map_publish_every_n = 5;
};

void loadCommonParams(const std::string& config_path, CommonParams& p);

// ─────────────────────────────────────────────────────────────────────────────
// Projection core.
//
// Projects the given points into one camera and reports each accepted point
// through `sink(point_index, packed_rgb)`.
//
// `T_cam_from_points` maps the points from whatever frame they are in straight
// into the camera frame:
//   ColoriseScan : camera-from-lidar (static extrinsic * motion compensation)
//   ColoriseMap  : (T_world_lidar * T_lidar_cam)^-1
//
// `candidates` lists the indices into P3 to consider — the height gate for
// scans, the kd-tree range cull for the map. It is always explicit: an "empty
// means all" convention would silently project everything on the frame where a
// filter happened to reject every point, which is exactly backwards.
// ─────────────────────────────────────────────────────────────────────────────
void projectAndSample(const std::vector<cv::Point3f>& P3,
                      const std::vector<int>&         candidates,
                      const cv::Mat&                  img,
                      const CameraCalib&              cam,
                      const Eigen::Matrix4d&          T_cam_from_points,
                      bool                            occlusion_check,
                      double                          occlusion_cell_px,
                      double                          occlusion_depth_tol,
                      const std::function<void(int, std::uint32_t)>& sink);

// ─────────────────────────────────────────────────────────────────────────────
// IMU orientation buffer (AHRS quaternion, SLERP-interpolated).
// ─────────────────────────────────────────────────────────────────────────────
class ImuBuffer {
public:
    void push(const sensor_msgs::ImuConstPtr& msg);
    bool interpolate(const ros::Time& t, Eigen::Quaterniond& q_out) const;
    bool empty() const { return buf_.empty(); }
    size_t size() const { return buf_.size(); }
    bool covers(const ros::Time& t) const;

private:
    std::deque<sensor_msgs::ImuConstPtr> buf_;
    double history_sec_ = 10.0;
};

// ─────────────────────────────────────────────────────────────────────────────
// Odometry trajectory. Fed either live from a topic or from a CSV exported by
// (time,x,y,z,qx,qy,qz,qw), one per line.
// ─────────────────────────────────────────────────────────────────────────────
class OdomTrajectory {
public:
    struct Sample {
        ros::Time          t;
        Eigen::Vector3d    p;
        Eigen::Quaterniond q;
    };

    // Returns the number of samples loaded; throws on an unreadable file.
    size_t loadCsv(const std::string& path);
    void   push(const nav_msgs::OdometryConstPtr& msg);

    // Interpolates (SLERP + LERP) at t. Fails if t falls outside the samples.
    bool interpolate(const ros::Time& t, Eigen::Matrix4d& T_out) const;
    // Nearest sample within max_dt; used by CompMode::NONE and as the IMU anchor.
    bool nearest(const ros::Time& t, double max_dt, Sample& s_out) const;

    bool   empty() const { return samples_.empty(); }
    size_t size()  const { return samples_.size(); }
    ros::Time front() const { return samples_.front().t; }
    ros::Time back()  const { return samples_.back().t; }

private:
    std::vector<Sample> samples_;   // kept sorted by time
    bool   live_          = false;
    double history_sec_   = 60.0;
};

Eigen::Matrix4d makeTransform(const Eigen::Quaterniond& q, const Eigen::Vector3d& p);

// Finds the message in `buffer` whose stamp is closest to `target`, within
// `max_offset` seconds. Returns nullptr if there is none.
template <typename Deque>
typename Deque::value_type findClosest(const Deque& buffer,
                                       const ros::Time& target,
                                       double max_offset) {
    typename Deque::value_type best = nullptr;
    double best_diff = std::numeric_limits<double>::max();
    for (const auto& msg : buffer) {
        double diff = std::fabs((msg->header.stamp - target).toSec());
        if (diff < best_diff && diff <= max_offset) {
            best      = msg;
            best_diff = diff;
        }
    }
    return best;
}

template <typename Deque>
void cleanOldMsgs(Deque& buffer, const ros::Time& latest, double history_sec = 2.0) {
    while (!buffer.empty() &&
           (latest - buffer.front()->header.stamp).toSec() > history_sec)
        buffer.pop_front();
}

#endif  // COLORISE_H
