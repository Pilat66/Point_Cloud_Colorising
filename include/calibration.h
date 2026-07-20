#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <string>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

// ─────────────────────────────────────────────────────────────────────────────
// Calibration / configuration loading.
//
// Everything is read from plain YAML via yaml-cpp. cv::FileStorage cannot parse
// configs/calibration.yaml (it rejects the nested matrix lists under cam0..cam3
// with "Incorrect indentation"), and nothing is looked up from the TF tree.
//
// Transform naming follows the Kalibr convention used in calibration.yaml:
//   T_A_B maps a point expressed in frame B into frame A.
// So T_os_sensor_forwardLeft_cam is "lidar from camera", and its inverse is the
// camera-from-lidar transform used for projection.
// ─────────────────────────────────────────────────────────────────────────────

struct CameraCalib {
    std::string name;             // friendly name, e.g. "forwardLeft"
    std::string image_topic;      // compressed image topic

    Eigen::Matrix4d T_cam_lidar = Eigen::Matrix4d::Identity();  // camera from lidar
    Eigen::Matrix4d T_lidar_cam = Eigen::Matrix4d::Identity();  // lidar from camera

    cv::Mat     K;                // 3x3 intrinsics
    cv::Mat     D;                // 1xN distortion coefficients
    std::string distortion_model; // "equidistant" (fisheye) or "plumb_bob"
    int         width  = 0;
    int         height = 0;

    // Fisheye edge rejection (see config.yaml for the rationale).
    double edge_margin_px     = 0.0;
    double max_view_angle_deg = 180.0;

    // Minimum distance from the camera centre, in metres. Points nearer than
    // this are not coloured: close geometry is where the fisheye model is least
    // accurate, and where a small extrinsic or timing error moves the projected
    // point furthest, so the colour sampled there is the least trustworthy.
    double min_camera_dist = 0.0;

    // Cached: cos of the half field-of-view actually accepted.
    double min_cos_view_angle = -1.0;
};

// Loads a YAML file, stripping any leading "%YAML:1.0"-style directive lines
// (Kalibr writes "%YAML:1.0" which yaml-cpp rejects as a malformed directive).
YAML::Node loadYamlLenient(const std::string& path);

// Reads a 4x4 matrix stored as a list of 4 row-lists.
Eigen::Matrix4d parseMatrix4d(const YAML::Node& node, const std::string& key);

// Expands "$(find <pkg>)" in a path string.
std::string expandRosPath(const std::string& path);

#endif  // CALIBRATION_H
