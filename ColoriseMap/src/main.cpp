// ColoriseMap — ROS-free colourisation of a pre-built point cloud map with
// camera images from the same run.
//
// Offline counterpart of the ColoriseMap ROS node (src/ColoriseMap.cpp). The
// map points are static and live in the map/world frame, so the only thing that
// varies per frame is where the camera was. The pose of the lidar at an image's
// own capture time is interpolated from a CSV of timestamped poses, the map is
// culled to a radius around the camera, and colour is accumulated per map point
// across frames — the pipeline the node runs on a live bag.
//
// The projection core is shared with the offline scan tool: the already
// validated sources of ../colorise are compiled into this target, so gates,
// fisheye projection, the occlusion z-buffer and RGB packing are the same code.
// What is app-specific here is the map/node semantics: several cameras paired by
// timestamp, the radius cull around each camera, min_color_frames accumulation
// and the end-of-run save.
//
// No ROS: photos come from directories (capture time in the filename, epoch
// seconds or nanoseconds), poses from a CSV, intrinsics/extrinsics from files.

#include "colorise.h"
#include "voxel_grid.hpp"
#include "cube_colourise.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace fs = std::filesystem;

// ─────────────────────────────────────────────────────────────────────────────
// Small formatting / filesystem helpers (same behaviour as colorise/main.cpp)
// ─────────────────────────────────────────────────────────────────────────────
static std::string withCommas(long long v) {
    std::string s = std::to_string(v);
    std::string out;
    int cnt = 0;
    for (std::size_t i = s.size(); i-- > 0;) {
        out.insert(0, s.substr(i, 1));
        if (++cnt % 3 == 0 && i > 0) out.insert(0, ",");
    }
    return out;
}

static void createParentDirs(const std::string& path) {
    std::error_code ec;
    fs::path p(path);
    if (p.has_parent_path()) fs::create_directories(p.parent_path(), ec);
}

// ─────────────────────────────────────────────────────────────────────────────
// One camera: --photos opens a camera group, the following --calib/--name
// belong to it. Repeat the pair for every camera.
// ────────────────────────────────────────────────────────────────────────────
struct CameraSpec {
    std::string name;
    std::string photos_dir;
    std::string calib_path;    // camera intrinsics + extrinsic (calib.json)
};

struct MapOptions {
    std::string cloud_path;       // map PCD/LAS, in the map frame
    std::string trajectory_path;  // lidar poses in that same frame
    std::string output_path;      // coloured map: .pcd or .las
    std::vector<CameraSpec> cameras;

    // Node parameters (config.yaml names kept: map_max_range, min_color_frames,
    // occlusion_*, compensation_mode, keep_uncolored_points).
    double      map_max_range       = 20.0;   // cull radius around the camera
    int         min_color_frames    = 1;      // observations before a point is kept
    std::string compensation        = "odom";  // none | odom
    bool        occlusion           = true;
    double      occlusion_cell_px   = 4.0;
    double      occlusion_depth_tol = 0.3;
    double      occlusion_max_depth  = std::numeric_limits<double>::infinity();  // --occlusion-max-depth (inf = off)
    int         occlusion_ray_margin = 1;     // --occlusion-ray-margin
    bool        keep_uncolored      = false;
    double      max_time_offset     = 0.05;   // camera pairing tolerance, s

    // Offline-only knobs (same names/semantics as colorise/).
    double      time_shift          = 0.0;    // applied to trajectory times
    double      time_tolerance      = 1.0;    // clamp at the trajectory ends
    double      edge_margin         = 0.0;
    double      max_view_angle_deg  = 75.0;
    double      min_camera_dist     = 2.0;
    std::string euler_order         = "xyz";
    std::string euler_units         = "auto";
    std::string extrinsic_name;
    std::string extrinsic_direction;
    bool        nearest_wins        = false;  // colour from the nearest camera only
    int         jobs                = 0;      // 0 = all cores

    // Cube mode (--cube): spatial 1 m buckets + per-point gps_time selection.
    double      cube_size           = 0.0;    // 0 = off (frame pipeline)
    double      score_w_time        = 1.0;    // --score-time-weight
    double      score_w_dist        = 1.0;    // --score-dist-weight
    double      score_t_ref         = 1.0;    // --score-time-scale, s
    double      score_d_ref         = 10.0;   // --score-dist-scale, m
    std::string debug1_path;              // --debug1 <csv> (cube mode only)
};

// ─────────────────────────────────────────────────────────────────────────────
// CLI
// ─────────────────────────────────────────────────────────────────────────────
static void printUsage() {
    std::fprintf(stderr,
        "Usage: colorise_map [options]\n"
        "\n"
        "Colourises a pre-built point cloud map from timestamped photos and a CSV\n"
        "of lidar poses (ROS-free counterpart of the ColoriseMap node).\n"
        "\n"
        "Camera groups: every --photos starts a new camera, and the --calib/--name\n"
        "that follow belong to it. Repeat the pair for each camera:\n"
        "  --photos <dir> --calib <json> [ --photos ... ]\n"
        "\n"
        "  --cloud <pcd|las>           map, in the world/map frame  [data/all_raw_points.pcd]\n"
        "  --trajectory <csv>          lidar poses in that frame    [data/trajectory.csv]\n"
        "  --output <pcd|las>          coloured map (.las = LAS 1.2) [coloured_map.pcd]\n"
        "  --photos <dir>              photos dir, capture time in the filename\n"
        "  --calib <json>              camera intrinsics + camera<->lidar extrinsic\n"
        "  --name <name>               camera label for the log     [camN]\n"
        "  --map-max-range <m>         cull radius around a camera   [20.0] (0 = off)\n"
        "  --min-color-frames <n>      observations before a point is kept [1]\n"
        "  --compensation {none|odom}                              [odom]\n"
        "  --max-time-offset <s>       camera pairing tolerance      [0.05]\n"
        "  --occlusion / --no-occlusion                            [on]\n"
        "  --occlusion-cell <px>       z-buffer cell                 [4.0]\n"
        "  --occlusion-depth-tol <m>   z-buffer tolerance            [0.3]\n"
        "  --occlusion-max-depth <m>   no-lidar-rays depth gate      [inf] (off)\n"
        "  --occlusion-ray-margin <n>  dilate that mask, cells       [1]\n"
        "  --keep-uncolored            write uncoloured points as black\n"
        "  --nearest-wins              colour each point only from the nearest camera\n"
        "  --cube <m>                  cube mode: 1 m buckets + per-point gps_time\n"
        "                              scoring (needs a LAS with timestamps)\n"
        "  --score-time-weight <w>     cube mode: weight of |dt|        [1.0]\n"
        "  --score-dist-weight <w>     cube mode: weight of distance    [1.0]\n"
        "  --score-time-scale <s>      cube mode: |dt| normaliser       [1.0]\n"
        "  --score-dist-scale <m>      cube mode: distance normaliser   [10.0]\n"
        "  --debug1 <csv>              cube mode: CSV \"which photo coloured\" \
which point (by gps_time)\n"
        "  --extrinsic-name <key>      exact/suffix key in calib.json\n"
        "  --extrinsic-direction {camera_from_lidar|lidar_from_camera}\n"
        "  --euler-order {xyz|zyx}     Euler rotation order          [xyz]\n"
        "  --euler-units {auto|deg|rad}                            [auto]\n"
        "  --time-shift <s>            shift applied to trajectory times [0]\n"
        "  --time-tolerance <s>        clamp tolerance at the ends   [1.0]\n"
        "  --min-camera-dist <m>       near-range gate               [2.0]\n"
        "  --max-view-angle <deg>      half-angle gate               [75.0]\n"
        "  --edge-margin <px>          image edge trim               [0.0]\n"
        "  --jobs <n>                  worker threads (0 = all cores) [0]\n"
        "  -h, --help                  this message\n");
}

static double parseDoubleValue(const char* s, const std::string& flag) {
    try {
        return std::stod(s);
    } catch (...) {
        throw std::runtime_error("cannot parse number for " + flag + ": '" + s + "'");
    }
}

static int parseLongValue(const char* s, const std::string& flag) {
    try {
        return std::stoi(s);
    } catch (...) {
        throw std::runtime_error("cannot parse integer for " + flag + ": '" + s + "'");
    }
}

static bool parseArgs(int argc, char** argv, MapOptions& o) {
    auto next_value = [&](int& i, const std::string& key) -> std::string {
        if (i + 1 >= argc)
            throw std::runtime_error("missing value for " + key);
        return argv[++i];
    };

    int cur = -1;  // index of the camera group being parsed; -1 before any --photos
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "-h" || arg == "--help") { printUsage(); std::exit(0); }
        if (arg.rfind("--", 0) != 0)
            throw std::runtime_error("unexpected argument: '" + arg + "'");
        std::size_t eq = arg.find('=');
        std::string key = arg;
        std::string val;
        bool has_val = false;
        if (eq != std::string::npos) {
            key = arg.substr(0, eq);
            val = arg.substr(eq + 1);
            has_val = true;
        }
        auto value = [&](const std::string& flag) -> std::string {
            return has_val ? val : next_value(i, flag);
        };

        if (key == "--photos") {
            o.cameras.push_back(CameraSpec{});
            cur = static_cast<int>(o.cameras.size()) - 1;
            o.cameras[static_cast<std::size_t>(cur)].photos_dir = value(key);
        } else if (key == "--calib") {
            if (cur < 0) throw std::runtime_error("--calib must follow --photos");
            o.cameras[static_cast<std::size_t>(cur)].calib_path = value(key);
        } else if (key == "--name") {
            if (cur < 0) throw std::runtime_error("--name must follow --photos");
            o.cameras[static_cast<std::size_t>(cur)].name = value(key);
        } else if (key == "--cloud") o.cloud_path = value(key);
        else if (key == "--trajectory") o.trajectory_path = value(key);
        else if (key == "--output") o.output_path = value(key);
        else if (key == "--map-max-range")
            o.map_max_range = parseDoubleValue(value(key).c_str(), key);
        else if (key == "--min-color-frames")
            o.min_color_frames = parseLongValue(value(key).c_str(), key);
        else if (key == "--max-time-offset")
            o.max_time_offset = parseDoubleValue(value(key).c_str(), key);
        else if (key == "--compensation") {
            const std::string v = value(key);
            if (v != "none" && v != "odom")
                throw std::runtime_error("--compensation must be none | odom (the node's imu "
                                         "mode needs an AHRS orientation, which these rigs do "
                                         "not publish; see README)");
            o.compensation = v;
        } else if (key == "--occlusion") o.occlusion = true;
        else if (key == "--no-occlusion") o.occlusion = false;
        else if (key == "--occlusion-cell")
            o.occlusion_cell_px = parseDoubleValue(value(key).c_str(), key);
        else if (key == "--occlusion-depth-tol")
            o.occlusion_depth_tol = parseDoubleValue(value(key).c_str(), key);
        else if (key == "--occlusion-max-depth")
            o.occlusion_max_depth = parseDoubleValue(value(key).c_str(), key);
        else if (key == "--occlusion-ray-margin") {
            o.occlusion_ray_margin = parseLongValue(value(key).c_str(), key);
            if (o.occlusion_ray_margin < 0)
                throw std::runtime_error("--occlusion-ray-margin must be >= 0");
        }
        else if (key == "--keep-uncolored") o.keep_uncolored = true;
        else if (key == "--nearest-wins") o.nearest_wins = true;
        else if (key == "--cube")
            o.cube_size = parseDoubleValue(value(key).c_str(), key);
        else if (key == "--score-time-weight")
            o.score_w_time = parseDoubleValue(value(key).c_str(), key);
        else if (key == "--score-dist-weight")
            o.score_w_dist = parseDoubleValue(value(key).c_str(), key);
        else if (key == "--score-time-scale")
            o.score_t_ref = parseDoubleValue(value(key).c_str(), key);
        else if (key == "--score-dist-scale")
            o.score_d_ref = parseDoubleValue(value(key).c_str(), key);
        else if (key == "--debug1")
            o.debug1_path = value(key);

        else if (key == "--extrinsic-name") o.extrinsic_name = value(key);
        else if (key == "--extrinsic-direction") {
            const std::string v = value(key);
            if (v != "camera_from_lidar" && v != "lidar_from_camera")
                throw std::runtime_error("--extrinsic-direction must be "
                                         "camera_from_lidar | lidar_from_camera");
            o.extrinsic_direction = v;
        } else if (key == "--euler-order") {
            const std::string v = value(key);
            if (v != "xyz" && v != "zyx")
                throw std::runtime_error("--euler-order must be xyz | zyx");
            o.euler_order = v;
        } else if (key == "--euler-units") {
            const std::string v = value(key);
            if (v != "auto" && v != "deg" && v != "rad")
                throw std::runtime_error("--euler-units must be auto | deg | rad");
            o.euler_units = v;
        } else if (key == "--time-shift")
            o.time_shift = parseDoubleValue(value(key).c_str(), key);
        else if (key == "--time-tolerance")
            o.time_tolerance = parseDoubleValue(value(key).c_str(), key);
        else if (key == "--min-camera-dist")
            o.min_camera_dist = parseDoubleValue(value(key).c_str(), key);
        else if (key == "--max-view-angle")
            o.max_view_angle_deg = parseDoubleValue(value(key).c_str(), key);
        else if (key == "--edge-margin")
            o.edge_margin = parseDoubleValue(value(key).c_str(), key);
        else if (key == "--jobs") {
            o.jobs = parseLongValue(value(key).c_str(), key);
            if (o.jobs < 0) throw std::runtime_error("--jobs must be >= 0");
        } else {
            throw std::runtime_error("unknown option: '" + key + "'");
        }
    }

    // Every camera group needs its calib.json; label the unnamed ones camN.
    for (std::size_t c = 0; c < o.cameras.size(); ++c) {
        const std::string nth = std::to_string(c + 1);
        if (o.cameras[c].calib_path.empty())
            throw std::runtime_error("camera group " + nth + " has no --calib");
        if (o.cameras[c].name.empty()) o.cameras[c].name = "cam" + nth;
    }
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Pose at a photo's capture time.
//
//   odom : interpolate the trajectory (SLERP + LERP) at the photo time, clamped
//          to the ends within --time-tolerance — the node's CompMode::ODOM.
//   none : nearest trajectory sample only, no interpolation — the node's
//          CompMode::NONE, the baseline that shows what the timestamp offset
//          costs.
//
// The node's CompMode::IMU is deliberately not ported: it SLERPs the AHRS
// orientation of the Imu message, and the rigs used in this repository publish
// a raw IMU with an identity orientation (see README).
// ─────────────────────────────────────────────────────────────────────────────
static bool poseAt(const Trajectory& tr, double pt, double tol, bool interpolate,
                   Eigen::Vector3d& p, Eigen::Quaterniond& q) {
    if (tr.empty()) return false;

    if (!interpolate) {
        const std::size_t at = static_cast<std::size_t>(
            std::lower_bound(tr.time.begin(), tr.time.end(), pt) - tr.time.begin());
        int    best    = -1;
        double best_dt = std::numeric_limits<double>::max();
        for (int k : {static_cast<int>(at) - 1, static_cast<int>(at)}) {
            if (k < 0 || k >= tr.size()) continue;
            const double dt = std::fabs(tr.time[static_cast<std::size_t>(k)] - pt);
            if (dt < best_dt) { best_dt = dt; best = k; }
        }
        if (best < 0 || best_dt > 0.5) return false;   // node: nearest(t, 0.5, s)
        p = tr.pos[static_cast<std::size_t>(best)];
        q = tr.quat[static_cast<std::size_t>(best)];
        return true;
    }

    if (pt < tr.minT - tol || pt > tr.maxT + tol) return false;
    const double tc = std::clamp(pt, tr.minT, tr.maxT);
    return tr.sampleAt(tc, p, q);
}

static bool computeTc(const Trajectory& tr, double pt, double tol, bool interpolate,
                      const Eigen::Matrix4d& T_cam_lidar, Eigen::Matrix4d& Tc) {
    Eigen::Vector3d    p;
    Eigen::Quaterniond q;
    if (!poseAt(tr, pt, tol, interpolate, p, q)) return false;
    const Eigen::Matrix4d Tw = makeTransform(q, p);            // world from lidar
    Tc = invertTransform(Tw * invertTransform(T_cam_lidar));   // camera from world
    return true;
}

// Index of the photo of `ph` closest to t, within max_dt; -1 when there is none.
// `ph` is sorted by time (listPhotos).
static int nearestPhotoIndex(const std::vector<Photo>& ph, double t, double max_dt) {
    if (ph.empty()) return -1;
    const std::size_t at = static_cast<std::size_t>(
        std::lower_bound(ph.begin(), ph.end(), t,
                         [](const Photo& a, double v) { return a.t < v; }) - ph.begin());
    int    best    = -1;
    double best_dt = std::numeric_limits<double>::max();
    for (int k : {static_cast<int>(at) - 1, static_cast<int>(at)}) {
        if (k < 0 || k >= static_cast<int>(ph.size())) continue;
        const double dt = std::fabs(ph[static_cast<std::size_t>(k)].t - t);
        if (dt < best_dt) { best_dt = dt; best = k; }
    }
    if (best < 0 || best_dt > max_dt) return -1;
    return best;
}

// Per-frame result produced by a worker thread. The heavy work (cull + imread +
// projection, for every camera of the frame) runs concurrently; committing —
// accumulation and logging — is done strictly in frame order, so the result and
// the log stay identical to a single-threaded run.
struct FrameResult {
    enum Status { OK, OUT_OF_SPAN, EMPTY_CAND, IMG_FAIL, ERROR };
    Status                status     = OK;
    std::size_t           index      = 0;
    double                t          = 0.0;   // drive time (first camera's photo)
    std::size_t           cand_count = 0;     // candidates, summed over cameras
    std::vector<uint32_t> rgb;
    std::vector<int>      idx;
    std::vector<double>   dist;   // camera-to-point, for --nearest-wins
    std::string           message;
};

// One frame: which photo of every camera takes part (index into photos[c]).
struct FrameJob {
    std::vector<int> pi;
};

// ─────────────────────────────────────────────────────────────────────────────
// Orchestration
// ─────────────────────────────────────────────────────────────────────────────
static int run(const MapOptions& o) {
    if (o.cameras.empty())
        throw std::runtime_error("no camera given: use "
                                 "--photos <dir> --calib <json>");
    if (!o.output_path.empty()) createParentDirs(o.output_path);

    const std::size_t ncam = o.cameras.size();

    // Cameras: intrinsics, extrinsic, photos (the shared ROS-free loaders).
    std::vector<CameraParams>       cams(ncam);
    std::vector<Eigen::Matrix4d>    T_cam_lidar(ncam, Eigen::Matrix4d::Identity());
    std::vector<std::vector<Photo>> photos(ncam);
    std::vector<std::string>        key_used(ncam), dir_used(ncam), cam_key(ncam);

    for (std::size_t c = 0; c < ncam; ++c) {
        const CameraSpec& s = o.cameras[c];
        loadCameraFromCalib(s.calib_path, cams[c], cam_key[c]);
        loadCalib(s.calib_path, o.extrinsic_name, o.extrinsic_direction,
                  T_cam_lidar[c], key_used[c], dir_used[c]);
        photos[c] = listPhotos(s.photos_dir);
        if (photos[c].empty())
            throw std::runtime_error("no timestamped images in " + s.photos_dir);
    }

    for (std::size_t c = 0; c < ncam; ++c) {
        const CameraParams& cam = cams[c];
        std::fprintf(stdout, "[camera] %s %s %dx%d (calib: %s) K=[[%.3f, %.3f, %.3f], "
                     "[%.3f, %.3f, %.3f], [%.3f, %.3f, %.3f]] dist=[",
                     o.cameras[c].name.c_str(), cam.model.c_str(), cam.width, cam.height,
                     cam_key[c].c_str(),
                     cam.K(0, 0), cam.K(0, 1), cam.K(0, 2),
                     cam.K(1, 0), cam.K(1, 1), cam.K(1, 2),
                     cam.K(2, 0), cam.K(2, 1), cam.K(2, 2));
        for (std::size_t i = 0; i < cam.dist.size(); ++i)
            std::fprintf(stdout, "%s%.5f", i ? ", " : "", cam.dist[i]);
        std::fprintf(stdout, "]\n");
    }
    for (std::size_t c = 0; c < ncam; ++c) {
        std::fprintf(stdout, "[extrinsic] %s: %s (%s), camera-from-lidar\n",
                     o.cameras[c].name.c_str(), key_used[c].c_str(), dir_used[c].c_str());
        for (int r = 0; r < 4; ++r)
            std::fprintf(stdout, "  [%9.4f %9.4f %9.4f %9.4f]\n",
                         T_cam_lidar[c](r, 0), T_cam_lidar[c](r, 1),
                         T_cam_lidar[c](r, 2), T_cam_lidar[c](r, 3));
    }

    Trajectory tr;
    loadTrajectory(o.trajectory_path, o.euler_order, o.euler_units, o.time_shift, tr);
    std::fprintf(stdout, "[trajectory] %d poses t=[%.3f, %.3f]\n",
                 tr.size(), tr.minT, tr.maxT);

    for (std::size_t c = 0; c < ncam; ++c) {
        const std::vector<Photo>& ph = photos[c];
        std::fprintf(stdout, "[photos] %s %zu t=[%.3f, %.3f]\n",
                     o.cameras[c].name.c_str(), ph.size(), ph.front().t, ph.back().t);
        if (ph.front().t > tr.maxT || ph.back().t < tr.minT)
            std::fprintf(stdout, "[WARN] photo times of '%s' and the trajectory do not overlap "
                         "-> colourisation will cover nothing; align with --time-shift or use "
                         "matching data\n", o.cameras[c].name.c_str());
    }

    // Map
    const auto t0 = std::chrono::steady_clock::now();
    Cloud cloud;
    loadCloud(o.cloud_path, cloud);
    std::fprintf(stdout, "[cloud] loading %s pts in %.1fs\n",
                 withCommas(static_cast<long long>(cloud.n())).c_str(),
                 std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count());

    // Finite filter. No height gate: the map is already in the world frame, and
    // the radius cull below is what keeps one frame affordable.
    {
        Cloud fin;
        fin.x.reserve(cloud.n());
        fin.y.reserve(cloud.n());
        fin.z.reserve(cloud.n());
        if (cloud.hasIntensity()) fin.intensity.reserve(cloud.n());
        for (std::size_t i = 0; i < cloud.n(); ++i) {
            if (!std::isfinite(cloud.x[i]) || !std::isfinite(cloud.y[i]) ||
                !std::isfinite(cloud.z[i]))
                continue;
            fin.x.push_back(cloud.x[i]);
            fin.y.push_back(cloud.y[i]);
            fin.z.push_back(cloud.z[i]);
            if (cloud.hasIntensity()) fin.intensity.push_back(cloud.intensity[i]);
        }
        cloud = std::move(fin);
    }
    const std::size_t n = cloud.n();
    if (n == 0)
        throw std::runtime_error("map has no finite points: " + o.cloud_path);

    std::fprintf(stdout, "[map] %s points  map_max_range=%.1f m  min_color_frames=%d  "
                 "compensation=%s  cameras=%zu\n",
                 withCommas(static_cast<long long>(n)).c_str(), o.map_max_range,
                 o.min_color_frames, o.compensation.c_str(), ncam);

    // Frame pairing: the first camera drives, and every other camera must have a
    // photo within --max-time-offset of that timestamp (the node's findClosest +
    // max_time_offset). This pass is single-threaded, so the pairing — and every
    // log line — stays independent of --jobs.
    const std::vector<Photo>& lead = photos[0];
    std::vector<FrameJob> frames;
    long long unpaired = 0;
    for (std::size_t k = 0; k < lead.size(); ++k) {
        FrameJob job;
        job.pi.assign(ncam, -1);
        job.pi[0] = static_cast<int>(k);
        bool ok = true;
        for (std::size_t c = 1; c < ncam && ok; ++c) {
            const int j = nearestPhotoIndex(photos[c], lead[k].t, o.max_time_offset);
            if (j < 0) ok = false;
            else       job.pi[c] = j;
        }
        if (!ok) { ++unpaired; continue; }
        frames.push_back(std::move(job));
    }
    if (unpaired)
        std::fprintf(stdout, "[WARN] %lld of %zu frames dropped: no partner camera within %.3f s\n",
                     unpaired, lead.size(), o.max_time_offset);
    if (frames.empty())
        throw std::runtime_error("no frame had every camera within " +
                                 std::to_string(o.max_time_offset) +
                                 " s — check the --photos directories and --max-time-offset");
    std::fprintf(stdout, "[frames] %zu usable (driven by '%s')\n",
                 frames.size(), o.cameras[0].name.c_str());

    // ── Parallel frame pipeline ────────────────────────────────────────────
    // Workers do the heavy per-frame work concurrently (voxel cull, imread,
    // projectAndSample — all read-only on the shared cloud/grid). Every frame is
    // committed strictly in frame order, so the accumulation, uniq_so_far and
    // the log lines are identical to a single-threaded run.
    const int njobs = o.jobs > 0
        ? o.jobs
        : static_cast<int>(std::max(1u, std::thread::hardware_concurrency()));
    std::fprintf(stdout, "[parallel] %d worker threads\n", njobs);

    const bool use_cull = o.map_max_range > 0.0;
    VoxelGrid grid;
    if (use_cull) grid.build(cloud, 1000000, njobs);

    // Shared candidate list for --map-max-range 0: built once, then only read.
    std::vector<int> all_idx;
    if (!use_cull) {
        all_idx.resize(n);
        for (std::size_t i = 0; i < n; ++i) all_idx[i] = static_cast<int>(i);
    }

    std::vector<double>   csum(3 * n, 0.0);
    std::vector<int32_t>  ccnt(n, 0);
    std::vector<uint32_t> out(n, 0);
    std::vector<char>     has(n, 0);
    // --nearest-wins: distance of the camera that coloured each point so far.
    std::vector<double>   best(n, std::numeric_limits<double>::infinity());
    long long uniq_so_far = 0;
    long long max_obs     = 0;

    const bool interp = (o.compensation != "none");

    std::mutex              commit_mu;
    std::condition_variable commit_cv;
    std::size_t             next_commit = 0;              // guarded by commit_mu
    bool                    pipeline_error = false;       // guarded by commit_mu
    std::string             pipeline_error_msg;           // guarded by commit_mu
    std::atomic<std::size_t> next_dispatch{0};            // frames grabbed in order

    auto commit = [&](const FrameResult& res) {           // called with commit_mu held
        if (res.status == FrameResult::ERROR) {
            pipeline_error     = true;
            pipeline_error_msg = res.message;
        } else if (res.status == FrameResult::OUT_OF_SPAN) {
            std::fprintf(stdout,
                "[frame %zu] t=%.3f outside trajectory span (tol %.2fs) -> skipped\n",
                res.index + 1, res.t, o.time_tolerance);
        } else if (res.status == FrameResult::EMPTY_CAND) {
            std::fprintf(stdout,
                "[frame %zu] no points within %.0f m -> skipped\n",
                res.index + 1, o.map_max_range);
        } else if (res.status == FrameResult::IMG_FAIL) {
            std::fprintf(stdout,
                "[frame %zu] t=%.3f image failed to load -> skipped\n",
                res.index + 1, res.t);
        } else if (!res.idx.empty()) {
            for (std::size_t q = 0; q < res.idx.size(); ++q) {
                const int      i = res.idx[q];
                const uint32_t p = res.rgb[q];
                if (o.nearest_wins) {
                    // keep the observation from the nearest camera; strict '<'
                    // makes the result independent of the frame order/threads
                    if (ccnt[i] == 0) ++uniq_so_far;
                    ccnt[i] += 1;
                    if (ccnt[i] > max_obs) max_obs = ccnt[i];
                    if (res.dist[q] < best[i]) {
                        has[i] = 1;              // provisional; recount at the end
                        out[i] = p;
                        best[i] = res.dist[q];
                    }
                    continue;
                }
                csum[3 * i + 0] += (p >> 16) & 0xFF;
                csum[3 * i + 1] += (p >> 8) & 0xFF;
                csum[3 * i + 2] += p & 0xFF;
                if (ccnt[i] == 0) ++uniq_so_far;
                ccnt[i] += 1;
                if (ccnt[i] > max_obs) max_obs = ccnt[i];
            }
            std::fprintf(stdout,
                "[frame %zu] t=%.3f coloured=%s cand=%s uniq_so_far=%s\n",
                res.index + 1, res.t,
                withCommas(static_cast<long long>(res.idx.size())).c_str(),
                withCommas(static_cast<long long>(res.cand_count)).c_str(),
                withCommas(uniq_so_far).c_str());
        }
    };

auto worker = [&]() {
        for (;;) {
            const std::size_t k = next_dispatch.fetch_add(1);
            if (k >= frames.size()) break;
            {
                std::unique_lock<std::mutex> lk(commit_mu);
                if (pipeline_error) break;      // abort the remaining frames
            }

            const FrameJob& job = frames[k];
            FrameResult     res;
            res.index = k;
            res.t     = photos[0][static_cast<std::size_t>(job.pi[0])].t;

            std::size_t pose_missing = 0;
            bool        img_fail     = false;
            try {
                for (std::size_t c = 0; c < ncam; ++c) {
                    const Photo& id = photos[c][static_cast<std::size_t>(job.pi[c])];

                    // The pose is taken at the photo's own capture time, exactly
                    // like the node does per camera.
                    Eigen::Matrix4d Tc;
                    if (!computeTc(tr, id.t, o.time_tolerance, interp, T_cam_lidar[c], Tc)) {
                        ++pose_missing;
                        continue;
                    }

                    // Camera centre in the world frame = Tc^-1 * [0 0 0 1]
                    const Eigen::Vector4d cam_h =
                        Tc.inverse() * Eigen::Vector4d(0.0, 0.0, 0.0, 1.0);

                    std::vector<int> cand;
                    if (use_cull) {
                        grid.radiusQuery(cloud, cam_h[0], cam_h[1], cam_h[2],
                                         o.map_max_range, cand);
                        res.cand_count += cand.size();
                        if (cand.empty()) continue;
                    } else {
                        res.cand_count += n;
                    }
                    const std::vector<int>& cand_ref = use_cull ? cand : all_idx;

                    const cv::Mat img = cv::imread(id.path, cv::IMREAD_COLOR);
                    if (img.empty()) { img_fail = true; continue; }

                    std::vector<uint32_t> rgb;
                    std::vector<int>      idx;
                    std::vector<double>   dist;
                    projectAndSample(cloud, cand_ref, img, cams[c], Tc,
                                     o.edge_margin, o.max_view_angle_deg, o.min_camera_dist,
                                     o.occlusion, o.occlusion_cell_px, o.occlusion_depth_tol,
                                     o.occlusion_max_depth, o.occlusion_ray_margin,
                                     rgb, idx, dist);
                    res.rgb.insert(res.rgb.end(), rgb.begin(), rgb.end());
                    res.idx.insert(res.idx.end(), idx.begin(), idx.end());
                    res.dist.insert(res.dist.end(), dist.begin(), dist.end());
                }

                if (pose_missing == ncam)                 res.status = FrameResult::OUT_OF_SPAN;
                else if (img_fail && res.idx.empty())     res.status = FrameResult::IMG_FAIL;
                else if (res.cand_count == 0)             res.status = FrameResult::EMPTY_CAND;
            } catch (const std::exception& e) {
                // Never let an exception escape a worker thread.
                res.status  = FrameResult::ERROR;
                res.message = e.what();
                res.idx.clear();
                res.rgb.clear();
                res.dist.clear();
            }

            // Commit in frame order: wait until the queue reaches this frame.
            {
                std::unique_lock<std::mutex> lk(commit_mu);
                commit_cv.wait(lk, [&] { return next_commit == k || pipeline_error; });
                if (!pipeline_error) commit(res);
                ++next_commit;
                commit_cv.notify_all();
                if (pipeline_error) break;
            }
        }
    };

    std::vector<std::thread> pool;
    pool.reserve(static_cast<std::size_t>(njobs));
    for (int t = 0; t < njobs; ++t) pool.emplace_back(worker);
    for (std::thread& th : pool) th.join();

    if (pipeline_error) throw std::runtime_error(pipeline_error_msg);

    // Finalise: average the observations; a point is kept when at least
    // min_color_frames frames saw it (node: buildColouredCloud).
    for (std::size_t i = 0; i < n; ++i) {
        if (o.nearest_wins) {
            // Recompute from the observation count (commit marked has[]
            // provisionally on the first sight of the point).
            has[i] = (ccnt[i] >= o.min_color_frames) ? 1 : 0;
            continue;                       // out already holds the nearest colour
        }
        if (ccnt[i] < o.min_color_frames) continue;
        has[i] = 1;
        const double r = csum[3 * i + 0] / ccnt[i];
        const double g = csum[3 * i + 1] / ccnt[i];
        const double b = csum[3 * i + 2] / ccnt[i];
        const uint32_t R = static_cast<uint32_t>(std::nearbyint(r));
        const uint32_t G = static_cast<uint32_t>(std::nearbyint(g));
        const uint32_t B = static_cast<uint32_t>(std::nearbyint(b));
        out[i] = (R << 16) | (G << 8) | B;
    }

    long long kept = 0;
    for (std::size_t i = 0; i < n; ++i)
        if (has[i]) ++kept;

    if (kept == 0 && max_obs > 0 && o.min_color_frames > 1)
        std::fprintf(stdout, "\n[WARN] frames coloured points (max %s observations/point) but "
                     "nothing kept: --min-color-frames=%d is too high for this run. "
                     "Try --min-color-frames 1.\n",
                     withCommas(max_obs).c_str(), o.min_color_frames);
    if (kept == 0 && max_obs == 0)
        std::fprintf(stdout, "\n[WARN] nothing was coloured: check that the map, the trajectory "
                     "and the photos come from the same run (ranges above), then try "
                     "--time-shift.\n");

    std::fprintf(stdout, "\n==== RESULT ====\n");
    std::fprintf(stdout, "  total=%s  coloured=%s  (%.2f%%)\n",
                 withCommas(static_cast<long long>(n)).c_str(),
                 withCommas(kept).c_str(),
                 n ? 100.0 * static_cast<double>(kept) / static_cast<double>(n) : 0.0);
    if (!o.output_path.empty()) {
        // Uncoloured points go out black under --keep-uncolored, exactly like the
        // node's keep_uncolored_points.
        if (o.keep_uncolored) std::fill(has.begin(), has.end(), 1);
        saveCloud(o.output_path, cloud, has, out, njobs);
        std::fprintf(stdout, "  wrote %s\n", o.output_path.c_str());
    }
    return 0;
}

// Cube mode: переиспользует загрузчики colorise/, но раскрашивает по кубам и
// по времени точки (см. cube_colourise.hpp).
static int runCube(const MapOptions& o, const CameraSpec& cam) {
    CubeOptions c;
    c.cloud_path          = o.cloud_path;
    c.photos_dir          = cam.photos_dir;
    c.trajectory_path     = o.trajectory_path;
    c.calib_path          = cam.calib_path;
    c.output_path         = o.output_path;
    c.extrinsic_name      = o.extrinsic_name;
    c.extrinsic_direction = o.extrinsic_direction;
    c.euler_order         = o.euler_order;
    c.euler_units         = o.euler_units;
    c.time_shift          = o.time_shift;
    c.time_tolerance      = o.time_tolerance;
    c.cube_size           = o.cube_size;
    c.occlusion           = o.occlusion;
    c.occlusion_cell_px   = o.occlusion_cell_px;
    c.occlusion_depth_tol = o.occlusion_depth_tol;
    c.occlusion_max_depth  = o.occlusion_max_depth;
    c.occlusion_ray_margin = o.occlusion_ray_margin;
    c.min_camera_dist     = o.min_camera_dist;
    c.max_view_angle_deg  = o.max_view_angle_deg;
    c.max_range           = o.map_max_range;
    c.score_w_time        = o.score_w_time;
    c.score_w_dist        = o.score_w_dist;
    c.score_t_ref         = o.score_t_ref;
    c.score_d_ref         = o.score_d_ref;
    c.keep_uncolored      = o.keep_uncolored;
    c.debug1_path         = o.debug1_path;
    c.jobs                = o.jobs;
    return runCubeColourise(c);
}

int main(int argc, char** argv) {
    MapOptions o;
    try {
        parseArgs(argc, argv, o);
        if (o.cube_size > 0.0) {
            if (o.cameras.size() != 1)
                throw std::runtime_error(
                    "--cube works with exactly one camera "
                    "(--photos <dir> --calib <json>)");
            return runCube(o, o.cameras.front());
        }
        if (!o.debug1_path.empty())
            throw std::runtime_error(
                "--debug1 works only together with --cube");
        return run(o);
    } catch (const std::exception& e) {
        std::fprintf(stderr, "\n[error] exception: %s\n", e.what());
        return 1;
    }
}