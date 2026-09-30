#include "colorise.h"
#include "voxel_grid.hpp"

#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <limits>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

namespace fs = std::filesystem;

// ─────────────────────────────────────────────────────────────────────────────
// Small formatting / filesystem helpers
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
// Camera->world transform for a photo (Python maybe_T).
// ─────────────────────────────────────────────────────────────────────────────
static bool computeTc(const Trajectory& tr, double pt, double tol,
                      const Eigen::Matrix4d& T_cam_lidar, Eigen::Matrix4d& Tc) {
    if (pt < tr.minT - tol || pt > tr.maxT + tol) return false;
    const double tc = std::clamp(pt, tr.minT, tr.maxT);
    Eigen::Vector3d p;
    Eigen::Quaterniond q;
    if (!tr.sampleAt(tc, p, q)) return false;
    const Eigen::Matrix4d Tw = makeTransform(q, p);
    Tc = invertTransform(Tw * invertTransform(T_cam_lidar));
    return true;
}
// ─────────────────────────────────────────────────────────────────────────────
// CLI parsing (mirrors the argparse surface of colorise_offline.py)
// ─────────────────────────────────────────────────────────────────────────────
static void printUsage() {
    std::fprintf(stderr,
        "Usage: colorise_offline [options]\n"
        "\n"
        "Offline point-cloud colourisation from timestamped photos + a lidar\n"
        "trajectory. C++ port of colorise_offline.py (standalone, no ROS).\n"
        "\n"
        "  --cloud <pcd>             input point cloud        [data/all_raw_points.pcd]\n"
        "  --photos <dir>            dir of timestamped photos [data]\n"
        "  --trajectory <csv>        lidar poses              [data/trajectory.csv]\n"
        "  --calib <json>            camera + extrinsic: intrinsics (\"camera\" block)\n"
        "                            and camera<->lidar transform [data/calib.json]\n"
        "  --output <pcd|las>        coloured cloud (.las=LAS 1.2) [coloured.pcd]\n"
        "  --extrinsic-name <key>    exact/suffix key in calib.json\n"
        "  --extrinsic-direction {camera_from_lidar|lidar_from_camera}\n"
        "  --euler-order {xyz|zyx}   Euler rotation order     [xyz]\n"
        "  --euler-units {auto|deg|rad}                        [auto]\n"
        "  --time-shift <s>          photo time offset        [0]\n"
        "  --time-tolerance <s>      anchor tolerance outside span [1.0]\n"
        "  --max-range <m>           radius cull around camera [20.0]\n"
        "  --min-camera-dist <m>     near-range gate          [2.0]\n"
        "  --max-view-angle <deg>    half-angle gate          [75.0]\n"
        "  --edge-margin <px>        image edge trim          [0.0]\n"
        "  --occlusion / --no-occlusion                        [on]\n"
        "  --occlusion-cell <px>     z-buffer cell            [4.0]\n"
        "  --occlusion-depth-tol <m> z-buffer tolerance       [0.3]\n"
        "  --occlusion-max-depth <m> no-lidar-rays depth gate  [40.0]\n"
        "  --occlusion-ray-margin <n> dilate that mask, cells  [1]\n"
        "  --min-color-frames <n>    min observing frames     [1]\n"
        "  --jobs <n>                worker threads (0 = all cores) [0]\n"
        "  --keep-uncolored          keep black points in output\n"
        "  --first-wins              keep first observed colour\n"
        "  --nearest-wins            colour each point only from the nearest camera\n"
        "  --max-lidar-z <m>         height gate (inf=off)    [inf]\n"
        "  -h, --help                this message\n");
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

static bool parseArgs(int argc, char** argv, Options& o) {
    auto next_value = [&](int& i, const std::string& key) -> std::string {
        if (i + 1 >= argc)
            throw std::runtime_error("missing value for " + key);
        return argv[++i];
    };

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
        auto num_value = [&](const std::string& flag) -> std::string {
            return has_val ? val : next_value(i, flag);
        };
        if (key == "--cloud") o.cloud_path = num_value(key);
        else if (key == "--photos") o.photos_dir = num_value(key);
        else if (key == "--trajectory") o.trajectory_path = num_value(key);
        else if (key == "--calib") o.calib_path = num_value(key);
        else if (key == "--output") o.output_path = num_value(key);
        else if (key == "--extrinsic-name") o.extrinsic_name = num_value(key);
        else if (key == "--extrinsic-direction") {
            const std::string v = num_value(key);
            if (v != "camera_from_lidar" && v != "lidar_from_camera")
                throw std::runtime_error("--extrinsic-direction must be "
                                         "camera_from_lidar | lidar_from_camera");
            o.extrinsic_direction = v;
        } else if (key == "--euler-order") {
            const std::string v = num_value(key);
            if (v != "xyz" && v != "zyx")
                throw std::runtime_error("--euler-order must be xyz | zyx");
            o.euler_order = v;
        } else if (key == "--euler-units") {
            const std::string v = num_value(key);
            if (v != "auto" && v != "deg" && v != "rad")
                throw std::runtime_error("--euler-units must be auto | deg | rad");
            o.euler_units = v;
        } else if (key == "--time-shift")
            o.time_shift = parseDoubleValue(num_value(key).c_str(), key);
        else if (key == "--time-tolerance")
            o.time_tolerance = parseDoubleValue(num_value(key).c_str(), key);
        else if (key == "--max-range")
            o.max_range = parseDoubleValue(num_value(key).c_str(), key);
        else if (key == "--min-camera-dist")
            o.min_camera_dist = parseDoubleValue(num_value(key).c_str(), key);
        else if (key == "--max-view-angle")
            o.max_view_angle_deg = parseDoubleValue(num_value(key).c_str(), key);
        else if (key == "--edge-margin")
            o.edge_margin = parseDoubleValue(num_value(key).c_str(), key);
        else if (key == "--occlusion-cell")
            o.occlusion_cell_px = parseDoubleValue(num_value(key).c_str(), key);
        else if (key == "--occlusion-depth-tol")
            o.occlusion_depth_tol = parseDoubleValue(num_value(key).c_str(), key);
        else if (key == "--occlusion-max-depth")
            o.occlusion_max_depth = parseDoubleValue(num_value(key).c_str(), key);
        else if (key == "--occlusion-ray-margin") {
            o.occlusion_ray_margin = parseLongValue(num_value(key).c_str(), key);
            if (o.occlusion_ray_margin < 0)
                throw std::runtime_error("--occlusion-ray-margin must be >= 0");
        }
        else if (key == "--min-color-frames")
            o.min_color_frames = parseLongValue(num_value(key).c_str(), key);
        else if (key == "--jobs") {
            o.jobs = parseLongValue(num_value(key).c_str(), key);
            if (o.jobs < 0)
                throw std::runtime_error("--jobs must be >= 0");
        }
        else if (key == "--max-lidar-z")
            o.max_lidar_z = parseDoubleValue(num_value(key).c_str(), key);
        else if (key == "--occlusion") o.occlusion = true;
        else if (key == "--no-occlusion") o.occlusion = false;
        else if (key == "--keep-uncolored") o.keep_uncolored = true;
        else if (key == "--first-wins") o.first_wins = true;
        else if (key == "--nearest-wins") o.nearest_wins = true;
        else throw std::runtime_error("unknown option: '" + key + "'");
    }
    if (o.first_wins && o.nearest_wins)
        throw std::runtime_error(
            "--first-wins and --nearest-wins are mutually exclusive: pick one");
    return true;
}
// Per-frame result produced by a worker thread. Heavy work (cull + imread +
// projection) runs concurrently; committing — accumulation and logging — is
// done strictly in frame order so the output and the logs stay identical to
// the single-threaded / Python run.
struct FrameResult {
    enum Status { OK, OUT_OF_SPAN, EMPTY_CAND, IMG_FAIL, ERROR };
    Status status = OK;
    std::size_t index = 0;
    double t = 0.0;
    std::size_t cand_count = 0;
    std::vector<uint32_t> rgb;
    std::vector<int> idx;
    std::vector<double> dist;      // camera-to-point, for --nearest-wins
    std::string message;
};

// ─────────────────────────────────────────────────────────────────────────────
// Orchestration (mirrors colourise() in the Python script)
// ─────────────────────────────────────────────────────────────────────────────
static int run(const Options& o) {
    if (!o.output_path.empty()) createParentDirs(o.output_path);

    // Camera: intrinsics + image size from the calib.json camera block.
    CameraParams cam;
    std::string cam_key;
    loadCameraFromCalib(o.calib_path, cam, cam_key);
    std::fprintf(stdout, "[camera] %s %dx%d K=[[%.3f, %.3f, %.3f], [%.3f, %.3f, %.3f], "
                 "[%.3f, %.3f, %.3f]] dist=[", cam.model.c_str(), cam.width, cam.height,
                 cam.K(0, 0), cam.K(0, 1), cam.K(0, 2),
                 cam.K(1, 0), cam.K(1, 1), cam.K(1, 2),
                 cam.K(2, 0), cam.K(2, 1), cam.K(2, 2));
    for (std::size_t i = 0; i < cam.dist.size(); ++i)
        std::fprintf(stdout, "%s%.5f", i ? ", " : "", cam.dist[i]);
    std::fprintf(stdout, "] (calib: %s)\n", cam_key.c_str());

    // Extrinsic
    Eigen::Matrix4d T_cam_lidar;
    std::string key_used, dir_used;
    loadCalib(o.calib_path, o.extrinsic_name, o.extrinsic_direction,
              T_cam_lidar, key_used, dir_used);
    std::fprintf(stdout, "[extrinsic] %s (%s)\n", key_used.c_str(), dir_used.c_str());
    for (int r = 0; r < 4; ++r)
        std::fprintf(stdout, "  [%9.4f %9.4f %9.4f %9.4f]\n",
                     T_cam_lidar(r, 0), T_cam_lidar(r, 1),
                     T_cam_lidar(r, 2), T_cam_lidar(r, 3));

    // Trajectory
    Trajectory tr;
    loadTrajectory(o.trajectory_path, o.euler_order, o.euler_units,
                   o.time_shift, tr);
    std::fprintf(stdout, "[trajectory] %d poses t=[%.3f, %.3f]\n",
                 tr.size(), tr.minT, tr.maxT);

    // Photos
    std::vector<Photo> ph = listPhotos(o.photos_dir);
    if (ph.empty())
        throw std::runtime_error("no timestamped images in " + o.photos_dir);
    std::fprintf(stdout, "[photos] %zu t=[%.3f, %.3f]\n",
                 ph.size(), ph.front().t, ph.back().t);
    if (ph.front().t > tr.maxT || ph.back().t < tr.minT) {
        std::fprintf(stdout,
            "[WARN] photo times and trajectory do not overlap -> colourisation "
            "will cover nothing; align with --time-shift or use matching data\n");
    }

    // Cloud
    const auto t0 = std::chrono::steady_clock::now();
    Cloud cloud;
    loadCloud(o.cloud_path, cloud);
    std::fprintf(stdout, "[cloud] loading %s pts in %.1fs\n",
                 withCommas(static_cast<long long>(cloud.n())).c_str(),
                 std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count());

    // Finite + max-lidar-z filter
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
            if (std::isfinite(o.max_lidar_z) && cloud.z[i] > o.max_lidar_z)
                continue;
            fin.x.push_back(cloud.x[i]);
            fin.y.push_back(cloud.y[i]);
            fin.z.push_back(cloud.z[i]);
            if (cloud.hasIntensity()) fin.intensity.push_back(cloud.intensity[i]);
        }
        cloud = std::move(fin);
    }
    const std::size_t n = cloud.n();

    const int njobs = o.jobs > 0
        ? o.jobs
        : static_cast<int>(std::max(1u, std::thread::hardware_concurrency()));
    std::fprintf(stdout, "[parallel] %d worker threads\n", njobs);

    const bool use_cull = o.max_range > 0.0;
    VoxelGrid grid;
    if (use_cull) grid.build(cloud, 1000000, njobs);

    // Shared candidate list for --max-range 0: built once, then only read by
    // the workers (a per-frame copy would waste 4*n bytes per thread).
    std::vector<int> all_idx;
    if (!use_cull) {
        all_idx.resize(n);
        for (std::size_t i = 0; i < n; ++i) all_idx[i] = static_cast<int>(i);
    }

    std::vector<double> csum(3 * n, 0.0);
    std::vector<int32_t> ccnt(n, 0);
    std::vector<uint32_t> out(n, 0);
    std::vector<char> colored(n, 0);
    std::vector<char> has(n, 0);
    // --nearest-wins: distance of the camera that coloured each point so far.
    std::vector<double> best(n, std::numeric_limits<double>::infinity());
    long long uniq_so_far = 0;
    long long max_obs = 0;

    // ── Parallel frame pipeline ────────────────────────────────────────────
    // Workers do the heavy per-frame work concurrently (voxel cull, imread,
    // projectAndSample — all read-only on the shared cloud/grid). Every frame
    // is then committed strictly in frame order: accumulation, the
    // uniq_so_far counter and the log lines are therefore identical to the
    // single-threaded run (colour sums are exact integers, so even the
    // averaging mode is order-independent; first-wins requires the strict
    // frame order, which this commit gate provides).
    std::mutex commit_mu;
    std::condition_variable commit_cv;
    std::size_t next_commit = 0;                   // guarded by commit_mu
    bool pipeline_error = false;                   // guarded by commit_mu
    std::string pipeline_error_msg;                // guarded by commit_mu
    std::atomic<std::size_t> next_dispatch{0};     // frames are grabbed in order

    auto commit = [&](const FrameResult& res) {    // called with commit_mu held
        if (res.status == FrameResult::ERROR) {
            pipeline_error = true;
            pipeline_error_msg = res.message;
        } else if (res.status == FrameResult::OUT_OF_SPAN) {
            std::fprintf(stdout,
                "[frame %zu] t=%.3f outside trajectory span (tol %.2fs) -> skipped\n",
                res.index + 1, res.t, o.time_tolerance);
        } else if (res.status == FrameResult::EMPTY_CAND) {
            std::fprintf(stdout,
                "[frame %zu] no points within %.0f m -> skipped\n",
                res.index + 1, o.max_range);
        } else if (!res.idx.empty()) {
            if (o.first_wins) {
                for (std::size_t q = 0; q < res.idx.size(); ++q) {
                    const int i = res.idx[q];
                    if (!colored[i]) {
                        colored[i] = 1;
                        out[i] = res.rgb[q];
                        ++uniq_so_far;
                    }
                }
            } else if (o.nearest_wins) {
                // keep the observation from the nearest camera; strict '<' makes
                // the result independent of the frame order / threading
                for (std::size_t q = 0; q < res.idx.size(); ++q) {
                    const int i = res.idx[q];
                    if (ccnt[i] == 0) ++uniq_so_far;
                    ccnt[i] += 1;
                    if (ccnt[i] > max_obs) max_obs = ccnt[i];
                    if (res.dist[q] < best[i]) {
                        colored[i] = 1;
                        out[i]     = res.rgb[q];
                        best[i]    = res.dist[q];
                    }
                }
            } else {
                for (std::size_t q = 0; q < res.idx.size(); ++q) {
                    const int i = res.idx[q];
                    const uint32_t p = res.rgb[q];
                    csum[3 * i + 0] += (p >> 16) & 0xFF;
                    csum[3 * i + 1] += (p >> 8) & 0xFF;
                    csum[3 * i + 2] += p & 0xFF;
                    if (ccnt[i] == 0) ++uniq_so_far;
                    ccnt[i] += 1;
                    if (ccnt[i] > max_obs) max_obs = ccnt[i];
                }
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
            if (k >= ph.size()) break;
            {
                std::unique_lock<std::mutex> lk(commit_mu);
                if (pipeline_error) break;     // abort remaining frames
            }

            FrameResult res;
            res.index = k;
            res.t = ph[k].t;
            try {
                Eigen::Matrix4d Tc;
                if (!computeTc(tr, ph[k].t, o.time_tolerance, T_cam_lidar, Tc)) {
                    res.status = FrameResult::OUT_OF_SPAN;
                } else {
                    // Camera position in the world frame = Tc^-1 * [0 0 0 1]
                    const Eigen::Vector4d cam_h =
                        Tc.inverse() * Eigen::Vector4d(0.0, 0.0, 0.0, 1.0);

                    std::vector<int> cand;
                    if (use_cull) {
                        grid.radiusQuery(cloud, cam_h[0], cam_h[1], cam_h[2],
                                         o.max_range, cand);
                        res.cand_count = cand.size();
                        if (cand.empty()) res.status = FrameResult::EMPTY_CAND;
                    } else {
                        res.cand_count = n;
                    }

                    if (res.status == FrameResult::OK) {
                        const std::vector<int>& cand_ref = use_cull ? cand : all_idx;
                        const cv::Mat img =
                            cv::imread(ph[k].path, cv::IMREAD_COLOR);
                        if (img.empty()) {
                            res.status = FrameResult::IMG_FAIL;
                        } else {
                            projectAndSample(cloud, cand_ref, img, cam, Tc,
                                             o.edge_margin, o.max_view_angle_deg,
                                             o.min_camera_dist, o.occlusion,
                                             o.occlusion_cell_px,
                                             o.occlusion_depth_tol,
                                             o.occlusion_max_depth,
                                             o.occlusion_ray_margin,
                                             res.rgb, res.idx, res.dist);
                        }
                    }
                }
            } catch (const std::exception& e) {
                // Never let an exception escape a worker thread.
                res.status = FrameResult::ERROR;
                res.message = e.what();
                res.idx.clear();
                res.rgb.clear();
            }

            // Commit in frame order: wait until the queue reaches this frame.
            {
                std::unique_lock<std::mutex> lk(commit_mu);
                commit_cv.wait(lk,
                               [&] { return next_commit == k || pipeline_error; });
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

    // Finalise: averaging across frames, or the first colour observed.
    if (o.first_wins) {
        has = colored;
    } else if (o.nearest_wins) {
        for (std::size_t i = 0; i < n; ++i)
            if (ccnt[i] >= o.min_color_frames) has[i] = 1;  // out already holds it
    } else {
        for (std::size_t i = 0; i < n; ++i) {
            if (ccnt[i] >= o.min_color_frames) {
                has[i] = 1;
                const double r = csum[3 * i + 0] / ccnt[i];
                const double g = csum[3 * i + 1] / ccnt[i];
                const double b = csum[3 * i + 2] / ccnt[i];
                const uint32_t R = static_cast<uint32_t>(std::nearbyint(r));
                const uint32_t G = static_cast<uint32_t>(std::nearbyint(g));
                const uint32_t B = static_cast<uint32_t>(std::nearbyint(b));
                out[i] = (R << 16) | (G << 8) | B;
            }
        }
    }
    long long has_sum = 0;
    for (std::size_t i = 0; i < n; ++i)
        if (has[i]) ++has_sum;

    if (has_sum == 0 && max_obs > 0 && o.min_color_frames > 1) {
        std::fprintf(stdout,
            "\n[WARN] frames coloured points (max %s observations/point) but "
            "nothing kept: --min-color-frames=%d is too high for this run. "
            "Try --min-color-frames 1.\n",
            withCommas(max_obs).c_str(), o.min_color_frames);
    }
    std::fprintf(stdout, "\n==== RESULT ====\n");
    std::fprintf(stdout, "  total=%s  coloured=%s  (%.2f%%)\n",
                 withCommas(static_cast<long long>(n)).c_str(),
                 withCommas(has_sum).c_str(),
                 n ? 100.0 * static_cast<double>(has_sum) / static_cast<double>(n)
                   : 0.0);
    if (!o.output_path.empty()) {
        // Python: sel = has, unless --keep-uncolored keeps every point (black).
        if (o.keep_uncolored) std::fill(has.begin(), has.end(), 1);
        saveCloud(o.output_path, cloud, has, out, njobs);
        std::fprintf(stdout, "  wrote %s\n", o.output_path.c_str());
    }
    return 0;
}

int main(int argc, char** argv) {
    Options o;
    try {
        parseArgs(argc, argv, o);
        return run(o);
    } catch (const std::exception& e) {
        std::fflush(stdout);
        std::fprintf(stderr, "\n[error] exception: %s\n", e.what());
        return 1;
    }
}