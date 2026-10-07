#include "cube_colourise.hpp"
#include "colorise.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <exception>
#include <fstream>
#include <limits>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <vector>

#include <opencv2/opencv.hpp>

#include <filesystem>

namespace {

std::string withCommas(long long v) {
    std::string s = std::to_string(v), out;
    int cnt = 0;
    for (std::size_t i = s.size(); i-- > 0;) {
        out.insert(0, s.substr(i, 1));
        if (++cnt % 3 == 0 && i > 0) out.insert(0, ",");
    }
    return out;
}

// Простой parallel-for по диапазонам [0, n).
// Динамическая выдача блоков (атомарный счётчик): кубы разной плотности, поэтому
// статическое разбиение даёт перекос по потокам. Результат не зависит от
// распределения блоков: каждая точка принадлежит одному кубу и обновляется
// только своим потоком.
template <typename F>
void parallelDynamic(std::size_t count, int jobs, std::size_t chunk, F fn) {
    if (count == 0) return;
    if (jobs <= 1) { fn(0, std::size_t(0), count); return; }
    std::atomic<std::size_t> next{0};
    std::vector<std::thread> th;
    std::mutex mu;
    std::exception_ptr err;
    for (int t = 0; t < jobs; ++t) {
        th.emplace_back([&, t] {
            try {
                for (;;) {
                    const std::size_t lo = next.fetch_add(chunk);
                    if (lo >= count) break;
                    fn(t, lo, std::min(count, lo + chunk));
                }
            } catch (...) {
                std::lock_guard<std::mutex> lk(mu);
                if (!err) err = std::current_exception();
            }
        });
    }
    for (std::thread& x : th) x.join();
    if (err) std::rethrow_exception(err);
}

// fn(tid, lo, hi): tid нужен вызывающему для приватных буферов потока.
template <typename F>
void parallelChunks(std::size_t n, int jobs, F fn) {
    if (jobs <= 1 || n == 0) { fn(0, std::size_t(0), n); return; }
    const std::size_t per = (n + static_cast<std::size_t>(jobs) - 1) /
                            static_cast<std::size_t>(jobs);
    std::vector<std::thread> th;
    std::mutex mu;
    std::exception_ptr err;
    for (int t = 0; t < jobs; ++t) {
        const std::size_t lo = static_cast<std::size_t>(t) * per;
        const std::size_t hi = std::min(n, lo + per);
        if (lo >= hi) break;
        const int tid = t;
        th.emplace_back([&, lo, hi, tid] {
            try { fn(tid, lo, hi); }
            catch (...) { std::lock_guard<std::mutex> lk(mu); if (!err) err = std::current_exception(); }
        });
    }
    for (std::thread& x : th) x.join();
    if (err) std::rethrow_exception(err);
}

// Собственная проекция pinhole (radtan/plumb_bob) — та же арифметика, что в
// projectPinhole() из colorise/src/project.cpp.
struct Proj {
    double fx, fy, cx, cy;
    double k1 = 0, k2 = 0, p1 = 0, p2 = 0, k3 = 0;
    bool   fisheye = false;

    static Proj from(const CameraParams& cam) {
        Proj p;
        p.fx = cam.K(0, 0); p.fy = cam.K(1, 1);
        p.cx = cam.K(0, 2); p.cy = cam.K(1, 2);
        p.fisheye = (cam.model == "fisheye");
        if (cam.dist.size() > 0) p.k1 = cam.dist[0];
        if (cam.dist.size() > 1) p.k2 = cam.dist[1];
        if (cam.dist.size() > 2) p.p1 = cam.dist[2];
        if (cam.dist.size() > 3) p.p2 = cam.dist[3];
        if (cam.dist.size() > 4) p.k3 = cam.dist[4];
        return p;
    }

    bool operator()(double X, double Y, double Z, double& u, double& v) const {
        if (!(Z > 1e-6)) return false;
        const double x = X / Z, y = Y / Z;
        const double r2 = x * x + y * y;
        const double radial = 1.0 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2;
        const double xd = x * radial + 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x);
        const double yd = y * radial + p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y;
        if (fisheye) return false;                 // кубовый режим: только pinhole
        u = fx * xd + cx;
        v = fy * yd + cy;
        return std::isfinite(u) && std::isfinite(v);
    }
};

// Ключ куба: индексы по осям, сдвинутые в положительную область и упакованные
// по 21 биту (сцена ~ +/- 1 км при кубе 1 м).
uint64_t cubeKey(double x, double y, double z, double cs) {
    const int64_t ix = static_cast<int64_t>(std::floor(x / cs)) + (int64_t(1) << 20);
    const int64_t iy = static_cast<int64_t>(std::floor(y / cs)) + (int64_t(1) << 20);
    const int64_t iz = static_cast<int64_t>(std::floor(z / cs)) + (int64_t(1) << 20);
    const uint64_t a = static_cast<uint64_t>(ix) & 0x1FFFFF;
    const uint64_t b = static_cast<uint64_t>(iy) & 0x1FFFFF;
    const uint64_t c = static_cast<uint64_t>(iz) & 0x1FFFFF;
    return (a << 42) | (b << 21) | c;
}

void cubeDecode(uint64_t key, int64_t& ix, int64_t& iy, int64_t& iz) {
    ix = static_cast<int64_t>((key >> 42) & 0x1FFFFF) - (int64_t(1) << 20);
    iy = static_cast<int64_t>((key >> 21) & 0x1FFFFF) - (int64_t(1) << 20);
    iz = static_cast<int64_t>(key & 0x1FFFFF) - (int64_t(1) << 20);
}

struct Frame {
    std::size_t        pi = 0;                 // индекс фото
    double             t  = 0.0;               // время съёмки
    Eigen::Matrix4d    Tcw = Eigen::Matrix4d::Identity();  // camera from world
    Eigen::Vector3d    c   = Eigen::Vector3d::Zero();      // центр камеры (мир)
    Eigen::Vector3d    axis = Eigen::Vector3d::UnitZ();    // оптическая ось (мир)
};

}  // namespace

int runCubeColourise(const CubeOptions& o) {
    const auto t_start = std::chrono::steady_clock::now();
    std::setvbuf(stdout, nullptr, _IOLBF, 0);   // прогресс виден построчно

    // ── Камера, экстраинсик, траектория, снимки ────────────────────────────
    CameraParams cam;
    std::string  cam_key;
    loadCameraFromCalib(o.calib_path, cam, cam_key);
    if (cam.model == "fisheye")
        throw std::runtime_error("cube mode supports pinhole cameras only "
                                 "(camera model is fisheye)");
    Eigen::Matrix4d T_cam_lidar;
    std::string     ext_key, ext_dir;
    loadCalib(o.calib_path, o.extrinsic_name, o.extrinsic_direction,
              T_cam_lidar, ext_key, ext_dir);
    Trajectory tr;
    loadTrajectory(o.trajectory_path, o.euler_order, o.euler_units, o.time_shift, tr);
    const std::vector<Photo> ph = listPhotos(o.photos_dir);
    if (ph.empty()) throw std::runtime_error("no timestamped images in " + o.photos_dir);

    std::printf("[camera] %s %dx%d (calib: %s)\n", cam.model.c_str(), cam.width,
                cam.height, cam_key.c_str());
    std::printf("[extrinsic] %s (%s)\n", ext_key.c_str(), ext_dir.c_str());
    std::printf("[trajectory] %d poses t=[%.3f, %.3f]\n", tr.size(), tr.minT, tr.maxT);
    std::printf("[photos] %zu t=[%.3f, %.3f]\n", ph.size(), ph.front().t, ph.back().t);

    // ── Карта: LAS с gps_time ──────────────────────────────────────────────
    // Файл читается потоково дважды (счёт кубов, затем упаковка): «сырые»
    // массивы на всю карту в памяти не держатся.
    const double cs = o.cube_size;
    if (!(cs > 0.0)) throw std::runtime_error("--cube must be > 0");
    LasMeta meta;
    std::unordered_map<uint64_t, uint32_t> count;
    streamLasRaw(o.cloud_path, meta,
        [&](std::size_t idx0, std::size_t c, const int32_t* Xc, const int32_t* Yc,
            const int32_t* Zc, const uint16_t*, const double*) {
            if (!meta.has_time)
                throw std::runtime_error(
                    "cube mode needs a LAS with gps_time (point format "
                    "1/3/4/5/6..10): " + o.cloud_path);
            if (idx0 == 0) count.reserve(meta.n / 8 + 16);
            for (std::size_t k = 0; k < c; ++k)
                ++count[cubeKey(meta.sx * Xc[k] + meta.ox, meta.sy * Yc[k] + meta.oy,
                                meta.sz * Zc[k] + meta.oz, cs)];
        });
    const std::size_t n = meta.n;
    std::printf("[cloud] %s points, LAS fmt %d, gps_time present\n",
                withCommas(static_cast<long long>(n)).c_str(), meta.fmt);

    const int njobs = o.jobs > 0
        ? o.jobs
        : static_cast<int>(std::max(1u, std::thread::hardware_concurrency()));
    std::printf("[parallel] %d worker threads\n", njobs);

    // ── Позы кадров (снимки без позы пропускаются) ─────────────────────────
    std::vector<Frame> frames;
    for (std::size_t i = 0; i < ph.size(); ++i) {
        if (ph[i].t < tr.minT - o.time_tolerance || ph[i].t > tr.maxT + o.time_tolerance)
            continue;
        Eigen::Vector3d    p;
        Eigen::Quaterniond q;
        const double tc = std::clamp(ph[i].t, tr.minT, tr.maxT);
        if (!tr.sampleAt(tc, p, q)) continue;
        const Eigen::Matrix4d Tw  = makeTransform(q, p);
        const Eigen::Matrix4d Tcw = invertTransform(Tw * invertTransform(T_cam_lidar));
        const Eigen::Matrix4d Twc = invertTransform(Tcw);
        Frame f;
        f.pi   = i;
        f.t    = ph[i].t;
        f.Tcw  = Tcw;
        f.c    = Twc.block<3, 1>(0, 3);
        f.axis = Twc.block<3, 3>(0, 0).col(2);
        frames.push_back(f);
    }
    std::printf("[frames] %zu usable of %zu photos\n", frames.size(), ph.size());
    if (frames.empty())
        throw std::runtime_error("no photo has a pose inside the trajectory span");

    // ── Пространственный индекс: кубы cube_size, CSR ──────────────────────

    const auto t_idx = std::chrono::steady_clock::now();
    std::vector<uint64_t> keys;
    keys.reserve(count.size());
    for (const auto& kv : count) keys.push_back(kv.first);
    std::sort(keys.begin(), keys.end());            // детерминированный порядок
    const std::size_t ncubes = keys.size();

    std::unordered_map<uint64_t, uint32_t> cid;
    cid.reserve(ncubes * 2);
    for (std::size_t i = 0; i < ncubes; ++i) cid[keys[i]] = static_cast<uint32_t>(i);

    std::vector<uint64_t> start(ncubes + 1, 0);
    for (std::size_t i = 0; i < ncubes; ++i)
        start[i + 1] = start[i] + count[keys[i]];
    // Точки упаковываются в порядке кубов: обход куба становится последовательным
    // (одна запись 24 Б вместо обращений к пяти массивам). Память под исходные
    // массивы освобождается сразу после упаковки.
    std::vector<LasPacked> pts(n);
{
        std::vector<uint64_t> cur(start.begin(), start.end() - 1);
        LasMeta meta2;
        streamLasRaw(o.cloud_path, meta2,
            [&](std::size_t, std::size_t c, const int32_t* Xc, const int32_t* Yc,
                const int32_t* Zc, const uint16_t* ic, const double* tc) {
                for (std::size_t k = 0; k < c; ++k) {
                    const uint64_t key = cubeKey(meta.sx * Xc[k] + meta.ox,
                                                 meta.sy * Yc[k] + meta.oy,
                                                 meta.sz * Zc[k] + meta.oz, cs);
                    LasPacked& q = pts[cur[cid[key]]++];
                    q.t         = tc ? tc[k] : 0.0;
                    q.X         = Xc[k];
                    q.Y         = Yc[k];
                    q.Z         = Zc[k];
                    q.intensity = ic[k];
                }
            });
    }
    const double sx = meta.sx, sy = meta.sy, sz = meta.sz;
    const double ox = meta.ox, oy = meta.oy, oz = meta.oz;
    std::vector<Eigen::Vector3d> centres(ncubes);
    for (std::size_t i = 0; i < ncubes; ++i) {
        int64_t ix, iy, iz;
        cubeDecode(keys[i], ix, iy, iz);
        centres[i] = Eigen::Vector3d((ix + 0.5) * cs, (iy + 0.5) * cs, (iz + 0.5) * cs);
    }
    count.clear(); cid.clear();
    keys.clear(); keys.shrink_to_fit();
    std::printf("[cubes] %s cubes of %.3g m in %.1fs\n",
                withCommas(static_cast<long long>(ncubes)).c_str(), cs,
                std::chrono::duration<double>(std::chrono::steady_clock::now() - t_idx).count());

    // ── Накопители: лучший балл и цвет каждой точки ───────────────────────
    std::vector<float>    best(n, std::numeric_limits<float>::infinity());
    std::vector<uint32_t> out(n, 0);
    std::vector<char>     has(n, 0);
    std::atomic<long long> uniq{0};
    // --debug1: индекс фото (в списке ph), давшего цвет точке; хранится только
    // при включённой опции (+4 Б на точку), dt/dist/score пересчитываются при записи.
    const bool  debug1   = !o.debug1_path.empty();
    std::vector<uint32_t> dbg_photo(debug1 ? n : 0, 0xFFFFFFFFu);

    const double cos_view = (o.max_view_angle_deg < 180.0)
        ? std::cos(o.max_view_angle_deg * M_PI / 180.0) : -1.0;
    const double halfdiag = 0.5 * cs * std::sqrt(3.0);
    const double cell     = std::max(1.0, o.occlusion_cell_px);
    const int    gw       = static_cast<int>(std::ceil(cam.width / cell));
    const int    gh       = static_cast<int>(std::ceil(cam.height / cell));
    const Proj   proj     = Proj::from(cam);

    std::vector<uint32_t> seen;                     // кубы в конусе кадра
    std::vector<std::vector<uint32_t>> per_thread(static_cast<std::size_t>(njobs));
    // z-буфер: у каждого потока своя сетка (иначе гонка при записи минимума),
    // в конце кадра они сливаются детерминированно — сначала i-й поток, потом j-й.
    const std::size_t ncells = static_cast<std::size_t>(gw) * static_cast<std::size_t>(gh);
    std::vector<std::vector<float>> grid_thread(
        static_cast<std::size_t>(njobs), std::vector<float>(ncells));
    std::vector<float> zmin(ncells);
    long long frame_coloured_total = 0;

    for (std::size_t fi = 0; fi < frames.size(); ++fi) {
        const Frame& fr = frames[fi];
        const cv::Mat img = cv::imread(ph[fr.pi].path, cv::IMREAD_COLOR);
        if (img.empty()) {
            std::printf("[frame %zu] t=%.3f image failed to load -> skipped\n", fi + 1, fr.t);
            continue;
        }

        // (a) кубы, которые могут попасть в кадр: консервативный тест центра.
        for (auto& v : per_thread) v.clear();
        parallelChunks(ncubes, njobs, [&](int tid, std::size_t lo, std::size_t hi) {
            std::vector<uint32_t>& mine = per_thread[static_cast<std::size_t>(tid)];
            mine.reserve((hi - lo) / 4 + 16);
            for (std::size_t ci = lo; ci < hi; ++ci) {
                const Eigen::Vector3d d = centres[ci] - fr.c;
                const double dist = d.norm();
                if (dist + halfdiag < o.min_camera_dist) continue;
                if (o.max_range > 0.0 && dist - halfdiag > o.max_range) continue;
                const double dz = fr.axis.dot(d);
                if (!(dz > 0.0)) continue;
                if (cos_view > -1.0) {
                    const double margin = halfdiag / std::max(dist, halfdiag);
                    if (dz / std::max(dist, 1e-9) < cos_view - margin) continue;
                }
                mine.push_back(static_cast<uint32_t>(ci));
            }
        });
        seen.clear();
        for (const auto& v : per_thread) seen.insert(seen.end(), v.begin(), v.end());

        // (b) z-буфер кадра: минимальная глубина на ячейку (по всем видимым точкам)
        if (o.occlusion) {
            for (auto& g : grid_thread)
                std::fill(g.begin(), g.end(), std::numeric_limits<float>::infinity());
            parallelDynamic(seen.size(), njobs, 64, [&](int tid, std::size_t lo, std::size_t hi) {
                std::vector<float>& g = grid_thread[static_cast<std::size_t>(tid)];
                for (std::size_t k = lo; k < hi; ++k) {
                    const uint32_t ci = seen[k];
                    for (uint32_t j = start[ci]; j < start[ci + 1]; ++j) {
                        const LasPacked& q = pts[j];
                        const Eigen::Vector3d pw(sx * q.X + ox, sy * q.Y + oy, sz * q.Z + oz);
                        const Eigen::Vector3d pc = fr.Tcw.block<3, 3>(0, 0) * pw +
                                                   fr.Tcw.block<3, 1>(0, 3);
                        if (!(pc.z() > 1e-6)) continue;
                        const double d3 = pc.norm();
                        if (d3 < o.min_camera_dist) continue;
                        if (cos_view > -1.0 && pc.z() / d3 < cos_view) continue;
                        double u, v;
                        if (!proj(pc.x(), pc.y(), pc.z(), u, v)) continue;
                        const int iu = static_cast<int>(std::nearbyint(u));
                        const int iv = static_cast<int>(std::nearbyint(v));
                        if (iu < 0 || iu >= cam.width || iv < 0 || iv >= cam.height) continue;
                        const std::size_t c = static_cast<std::size_t>(iv / cell) * gw +
                                              static_cast<std::size_t>(iu / cell);
                        const float zf = static_cast<float>(pc.z());
                        if (zf < g[c]) g[c] = zf;
                    }
                }
            });
            // Слияние приватных сеток в общую (детерминированный порядок).
            std::copy(grid_thread.front().begin(), grid_thread.front().end(), zmin.begin());
            for (std::size_t t = 1; t < grid_thread.size(); ++t)
                for (std::size_t c = 0; c < ncells; ++c)
                    if (grid_thread[t][c] < zmin[c]) zmin[c] = grid_thread[t][c];
        }

        // (c) скоринг: цвет от кадра с минимальным баллом, без усреднения
        std::atomic<long long> updates{0};
        parallelDynamic(seen.size(), njobs, 64, [&](int, std::size_t lo, std::size_t hi) {
            long long local = 0;
            for (std::size_t k = lo; k < hi; ++k) {
                const uint32_t ci = seen[k];
                for (uint32_t j = start[ci]; j < start[ci + 1]; ++j) {
                    const LasPacked& q = pts[j];
                    const double dt = std::fabs(fr.t - q.t);
                    const double term_t = o.score_w_time * dt / o.score_t_ref;
                    if (term_t >= best[j]) continue;              // раннее отсечение
                    const Eigen::Vector3d pw(sx * q.X + ox, sy * q.Y + oy, sz * q.Z + oz);
                    const Eigen::Vector3d pc = fr.Tcw.block<3, 3>(0, 0) * pw +
                                               fr.Tcw.block<3, 1>(0, 3);
                    if (!(pc.z() > 1e-6)) continue;
                    const double d3 = pc.norm();
                    if (d3 < o.min_camera_dist) continue;
                    if (cos_view > -1.0 && pc.z() / d3 < cos_view) continue;
                    double u, v;
                    if (!proj(pc.x(), pc.y(), pc.z(), u, v)) continue;
                    const int iu = static_cast<int>(std::nearbyint(u));
                    const int iv = static_cast<int>(std::nearbyint(v));
                    if (iu < 0 || iu >= cam.width || iv < 0 || iv >= cam.height) continue;
                    if (o.occlusion) {
                        const std::size_t c = static_cast<std::size_t>(iv / cell) * gw +
                                              static_cast<std::size_t>(iu / cell);
                        if (static_cast<float>(pc.z()) >
                            zmin[c] + static_cast<float>(o.occlusion_depth_tol))
                            continue;
                    }
                    const double score = term_t +
                        o.score_w_dist * std::min(1.0, d3 / o.score_d_ref);
                    if (score < best[j]) {
                        best[j] = static_cast<float>(score);
                        if (debug1) dbg_photo[j] = static_cast<uint32_t>(fr.pi);
                        const int su = static_cast<int>(std::nearbyint(
                            static_cast<double>(iu) * img.cols / cam.width));
                        const int sv = static_cast<int>(std::nearbyint(
                            static_cast<double>(iv) * img.rows / cam.height));
                        const int cu = std::max(0, std::min(su, img.cols - 1));
                        const int cv = std::max(0, std::min(sv, img.rows - 1));
                        const cv::Vec3b bgr = img.at<cv::Vec3b>(cv, cu);
                        out[j] = (static_cast<uint32_t>(bgr[2]) << 16) |
                                 (static_cast<uint32_t>(bgr[1]) << 8) |
                                 static_cast<uint32_t>(bgr[0]);
                        if (!has[j]) { has[j] = 1; ++uniq; }
                        ++local;
                    }
                }
            }
            updates += local;
        });
        // счётчик лучших обновлений не равен числу впервые окрашенных точек,
        // поэтому печатаем и оценку по updates только как «обновлений»
        frame_coloured_total += updates.load();
        if ((fi + 1) % 20 == 0 || fi + 1 == frames.size())
            std::printf("[frame %zu/%zu] t=%.3f cubes=%s updates=%s uniq=%s\n",
                        fi + 1, frames.size(), fr.t,
                        withCommas(static_cast<long long>(seen.size())).c_str(),
                        withCommas(updates.load()).c_str(),
                        withCommas(uniq.load()).c_str());
    }

    long long coloured = 0;
    for (std::size_t i = 0; i < n; ++i)
        if (has[i]) ++coloured;
    std::printf("\n==== RESULT ====\n");
    std::printf("  total=%s  coloured=%s  (%.2f%%)\n",
                withCommas(static_cast<long long>(n)).c_str(),
                withCommas(coloured).c_str(),
                n ? 100.0 * static_cast<double>(coloured) / static_cast<double>(n) : 0.0);

    if (!o.output_path.empty()) {
        std::vector<char> keep(n, 0);
        for (std::size_t i = 0; i < n; ++i)
            keep[i] = (o.keep_uncolored || has[i]) ? 1 : 0;
        saveLasPacked7(o.output_path, pts, keep, out, sx, sy, sz, ox, oy, oz);
        std::printf("  wrote %s (LAS 1.4 fmt 7, RGB + gps_time, %s points)\n",
                    o.output_path.c_str(),
                    withCommas(o.keep_uncolored ? static_cast<long long>(n) : coloured).c_str());
    }

    // ── --debug1: CSV «какой кадр дал цвет какой точке» ────────────────────
    if (debug1) {
        const auto t_dbg = std::chrono::steady_clock::now();
        std::ofstream df(o.debug1_path, std::ios::binary);
        if (!df.is_open())
            throw std::runtime_error("cannot write debug1 csv: " + o.debug1_path);
        for (std::size_t i = 0; i < ph.size(); ++i)
            df << "# photo " << i << " = "
               << std::filesystem::path(ph[i].path).filename().string() << '\n';
        // gps_time в этой карте квантован (~0.09 с, у точки есть «двойники»),
        // поэтому точка однозначно определяется парой (point, gps_time), где
        // point — номер строки в выходном LAS (порядок совпадает с порядком
        // этого CSV).
        df << "point,gps_time,photo,dt,dist,score\n";
        std::vector<int> frame_of_photo(ph.size(), -1);
        for (std::size_t fi = 0; fi < frames.size(); ++fi)
            frame_of_photo[frames[fi].pi] = static_cast<int>(fi);
        std::string rowbuf;
        rowbuf.reserve(1 << 20);
        char line[256];
        long long rows = 0;
        std::size_t out_row = 0;               // номер точки в выходном LAS
        for (std::size_t k = 0; k < n; ++k) {
            const bool written = o.keep_uncolored || has[k];
            const std::size_t this_row = out_row;   // номер ЭТОЙ точки в выходе
            if (written) ++out_row;
            if (!has[k]) continue;
            const uint32_t pidx = dbg_photo[k];
            const int fi = frame_of_photo[pidx];
            if (fi < 0) continue;                       // не бывает: победитель всегда с позой
            const Frame& fr = frames[static_cast<std::size_t>(fi)];
            const LasPacked& q = pts[k];
            const double dtp = std::fabs(fr.t - q.t);
            const double dx = sx * q.X + ox - fr.c.x();
            const double dy = sy * q.Y + oy - fr.c.y();
            const double dzz = sz * q.Z + oz - fr.c.z();
            const double dist = std::sqrt(dx * dx + dy * dy + dzz * dzz);
            const double score = o.score_w_time * dtp / o.score_t_ref +
                                 o.score_w_dist * std::min(1.0, dist / o.score_d_ref);
            std::snprintf(line, sizeof(line), "%zu,%.9f,%u,%.6f,%.6f,%.9f\n",
                          this_row, q.t, pidx, dtp, dist, score);
            rowbuf.append(line);
            if (rowbuf.size() >= (1u << 20)) {
                df << rowbuf;
                rowbuf.clear();
            }
            ++rows;
        }
        df << rowbuf;
        if (!df)
            throw std::runtime_error("cannot write debug1 csv (disk full?): " + o.debug1_path);
        std::printf("[debug1] %s rows -> %s (%.1fs)\n",
                    withCommas(rows).c_str(), o.debug1_path.c_str(),
                    std::chrono::duration<double>(std::chrono::steady_clock::now() - t_dbg).count());
    }
    std::printf("[time] %.1fs total\n",
                std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start).count());
    return 0;
}
