// colorise.h — C++ port of colorise_offline.py (standalone, ROS-free).
// The pipeline mirrors the Python reference 1:1: gates, fisheye projection,
// z-buffer occlusion, frame averaging.
#pragma once

#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

// CLI options. Defaults match colorise_offline.py exactly.
struct Options {
    std::string cloud_path      = "data/all_raw_points.pcd";
    std::string photos_dir      = "data";
    std::string trajectory_path = "data/trajectory.csv";
    std::string camera_path     = "data/camera.yaml";
    std::string calib_path      = "data/calib.json";
    std::string output_path     = "coloured.pcd";

    std::string extrinsic_name      = "";   // --extrinsic-name
    std::string extrinsic_direction = "";   // "" | camera_from_lidar | lidar_from_camera
    std::string euler_order  = "xyz";       // xyz | zyx
    std::string euler_units  = "auto";      // auto | deg | rad

    double time_shift          = 0.0;
    double time_tolerance      = 1.0;
    double max_range           = 20.0;
    double min_camera_dist     = 2.0;
    double max_view_angle_deg  = 75.0;
    double edge_margin         = 0.0;
    bool   occlusion           = true;
    double occlusion_cell_px   = 4.0;
    double occlusion_depth_tol = 0.3;
    int    min_color_frames    = 1;
    bool   keep_uncolored      = false;
    bool   first_wins          = false;
    double max_lidar_z         = std::numeric_limits<double>::infinity();
    int    jobs                = 0;    // worker threads; 0 = hardware_concurrency
};
// Camera intrinsics, loaded from camera.yaml (Kalibr-style keys cam_fx..cam_dN).
struct CameraParams {
    std::string model = "pinhole";          // "pinhole" | "fisheye"
    int width  = 1600;
    int height = 1300;
    Eigen::Matrix3d K = Eigen::Matrix3d::Zero();       // [[fx,0,cx],[0,fy,cy],[0,0,1]]
    std::vector<double> dist;                          // cam_d0..d5 / distortion_coeffs
};

// Lidar trajectory. `time` sorted ascending; `quat` normalised (w,x,y,z).
struct Trajectory {
    std::vector<double> time;
    std::vector<Eigen::Vector3d>    pos;
    std::vector<Eigen::Quaterniond> quat;
    std::vector<std::string> columns;

    double minT = 0.0;
    double maxT = 0.0;

    int  size() const { return static_cast<int>(time.size()); }
    bool empty() const { return time.empty(); }

    // Pose at t (linear position + SLERP). False when t is outside [minT,maxT].
    bool sampleAt(double t, Eigen::Vector3d& p, Eigen::Quaterniond& q) const;
};

// Point cloud, already filtered (finite coords, --max-lidar-z).
struct Cloud {
    std::vector<double> x, y, z;
    std::vector<float>  intensity;              // empty when the field is absent

    size_t n() const { return x.size(); }
    bool hasIntensity() const { return !intensity.empty(); }
};

// A timestamped photo: capture time (seconds, epoch) derived from the filename.
struct Photo {
    double t;
    std::string path;
};

// ─────────────────────────────────────────────────────────────────────────────
// 4x4 helpers: built from quaternion+position, and inverted (rigid).
// ─────────────────────────────────────────────────────────────────────────────
Eigen::Matrix4d makeTransform(const Eigen::Quaterniond& q, const Eigen::Vector3d& p);
Eigen::Matrix4d makeTransform(const Eigen::Matrix3d& R, const Eigen::Vector3d& p);
Eigen::Matrix4d invertTransform(const Eigen::Matrix4d& T);   // rigid inversion

// Normalises a quaternion (Eigen order w,x,y,z); the identity quaternion when
// the norm is ~0 (mirrors Python quat_normalize).
Eigen::Quaterniond quatNormalize(const Eigen::Quaterniond& q);

// ─────────────────────────────────────────────────────────────────────────────
// I/O (src/io.cpp). All loaders throw std::runtime_error on failure.
// ─────────────────────────────────────────────────────────────────────────────
void loadCamera(const std::string& path, CameraParams& cam);

// Resolves the extrinsic from calib.json, honouring the Python semantics:
// recursive key search, name-based direction auto-detection, 4/7/12/16 layouts.
// Returns T_cam_lidar (camera-from-lidar). key_used/direction_used mirror the
// Python printout.
void loadCalib(const std::string& path, const std::string& name,
               const std::string& direction, Eigen::Matrix4d& T_cam_lidar,
               std::string& key_used, std::string& direction_used);

void loadTrajectory(const std::string& path, const std::string& euler_order,
                    const std::string& euler_units, double time_shift,
                    Trajectory& tr);

// Reads a binary PCD or an uncompressed .las (LAS 1.0-1.4, point formats
// 0-10; .laz is rejected, like in the Python script without laszip).
void loadCloud(const std::string& path, Cloud& cloud, int jobs = 1);

// Writes the subset `keep` (parallel to cloud) as a binary XYZI+RGB PCD, or as
// LAS 1.2 point format 3 when the path ends in .las (.laz is rejected).
void saveCloud(const std::string& path, const Cloud& cloud,
               const std::vector<char>& keep, const std::vector<uint32_t>& rgb,
               int jobs = 1);

// Photos from a directory, timestamped from their filenames, sorted by time.
std::vector<Photo> listPhotos(const std::string& dir);

// Capture time in seconds drawn from the longest digit run in the filename.
double photoTimestampFromName(const std::string& stem);

// ─────────────────────────────────────────────────────────────────────────────
// Projection core (src/project.cpp).
//
// Projects the candidate points into the camera frame (T_cam_from_world) and
// samples the BGR image, mirroring project_and_sample() in the Python script:
//   z>0 / min_camera_dist / max_view_angle gates,
//   cv::projectPoints or cv::fisheye::projectPoints,
//   edge-margin rectangle check, z-buffer occlusion, pixel scaling for
//   mismatched image size.
// Every accepted candidate yields one packed RGB (PCL convention) and its
// index into `cloud`.
// ─────────────────────────────────────────────────────────────────────────────
void projectAndSample(const Cloud& cloud, const std::vector<int>& cand,
                      const cv::Mat& img, const CameraParams& cam,
                      const Eigen::Matrix4d& T_cam_from_world,
                      double edge_margin, double max_view_angle_deg,
                      double min_camera_dist, bool occlusion,
                      double occlusion_cell_px, double occlusion_depth_tol,
                      std::vector<uint32_t>& out_rgb, std::vector<int>& out_idx);