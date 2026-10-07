#include "colorise.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>

// ─────────────────────────────────────────────────────────────────────────────
// Quaternion / 4x4 helpers (mirrors colorise_offline.py).
// ─────────────────────────────────────────────────────────────────────────────

Eigen::Matrix4d makeTransform(const Eigen::Quaterniond& q, const Eigen::Vector3d& p) {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.topLeftCorner(3, 3) = q.toRotationMatrix();
    T.col(3).head(3) = p;
    return T;
}

Eigen::Matrix4d makeTransform(const Eigen::Matrix3d& R, const Eigen::Vector3d& p) {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.topLeftCorner(3, 3) = R;
    T.col(3).head(3) = p;
    return T;
}

Eigen::Matrix4d invertTransform(const Eigen::Matrix4d& T) {
    Eigen::Matrix4d out = Eigen::Matrix4d::Identity();
    const Eigen::Matrix3d R = T.topLeftCorner(3, 3);
    out.topLeftCorner(3, 3) = R.transpose();
    out.col(3).head(3) = -(R.transpose() * T.col(3).head(3));
    return out;
}

Eigen::Quaterniond quatNormalize(const Eigen::Quaterniond& q) {
    const double n = q.norm();
    if (n < 1e-12) return Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
    Eigen::Quaterniond out = q;
    out.coeffs() /= n;               // normalises all four components
    return out;
}

// Python quat_from_rpy; quaternion order = (w, x, y, z).
static Eigen::Quaterniond quatFromRpy(double roll, double pitch, double yaw,
                                      const std::string& order) {
    const double cr = std::cos(roll * 0.5), sr = std::sin(roll * 0.5);
    const double cp = std::cos(pitch * 0.5), sp = std::sin(pitch * 0.5);
    const double cy = std::cos(yaw * 0.5), sy = std::sin(yaw * 0.5);
    Eigen::Quaterniond q;
    if (order == "xyz") {
        q = Eigen::Quaterniond(
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy);
    } else if (order == "zyx") {
        q = Eigen::Quaterniond(
            cr * cp * cy + sr * sp * sy,
            cr * sp * cy - sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            sr * cp * cy + cr * sp * sy);
    } else {
        throw std::runtime_error("unknown --euler-order '" + order + "'");
    }
    return quatNormalize(q);
}

// Python quat_slerp, operating on quaternions in (w,x,y,z).
static Eigen::Quaterniond quatSlerp(const Eigen::Quaterniond& a0,
                                    const Eigen::Quaterniond& b0, double t) {
    Eigen::Quaterniond a = quatNormalize(a0);
    Eigen::Quaterniond b = quatNormalize(b0);
    const double wx1 = a.w(), xx1 = a.x(), xy1 = a.y(), xz1 = a.z();
    double wx2 = b.w(), xx2 = b.x(), xy2 = b.y(), xz2 = b.z();
    double d = wx1 * wx2 + xx1 * xx2 + xy1 * xy2 + xz1 * xz2;
    if (d < 0.0) { wx2 = -wx2; xx2 = -xx2; xy2 = -xy2; xz2 = -xz2; d = -d; }
    if (1.0 - d < 1e-6) {
        return quatNormalize(Eigen::Quaterniond(
            (1.0 - t) * wx1 + t * wx2,
            (1.0 - t) * xx1 + t * xx2,
            (1.0 - t) * xy1 + t * xy2,
            (1.0 - t) * xz1 + t * xz2));
    }
    const double th = std::acos(std::max(-1.0, std::min(1.0, d)));
    const double st = std::sin(th);
    const double s0 = std::sin((1.0 - t) * th) / st;
    const double s1 = std::sin(t * th) / st;
    return quatNormalize(Eigen::Quaterniond(
        s0 * wx1 + s1 * wx2, s0 * xx1 + s1 * xx2,
        s0 * xy1 + s1 * xy2, s0 * xz1 + s1 * xz2));
}

// ─────────────────────────────────────────────────────────────────────────────
// Trajectory sampling (Python Trajectory.sample_at).
// ─────────────────────────────────────────────────────────────────────────────

bool Trajectory::sampleAt(double t, Eigen::Vector3d& p, Eigen::Quaterniond& q) const {
    if (t < time.front() || t > time.back()) return false;
    // np.searchsorted(ts, t, side="right") == first index with time[i] > t
    size_t i = static_cast<size_t>(
        std::upper_bound(time.begin(), time.end(), t) - time.begin());
    if (i == 0) { p = pos[0]; q = quat[0]; return true; }
    if (i >= time.size()) { p = pos.back(); q = quat.back(); return true; }
    const size_t lo = i - 1, hi = i;
    const double dt = time[hi] - time[lo];
    const double a = (dt <= 1e-12) ? 0.0 : (t - time[lo]) / dt;
    p = (1.0 - a) * pos[lo] + a * pos[hi];
    q = quatSlerp(quat[lo], quat[hi], a);
    return true;
}
// ─────────────────────────────────────────────────────────────────────────────
// Euler unit auto-detection (Python _detect_units: 90th-percentile < pi → rad).
// ─────────────────────────────────────────────────────────────────────────────
static double percentile90(std::vector<double>& sorted_abs) {
    const double n = static_cast<double>(sorted_abs.size());
    if (n <= 1) return sorted_abs.front();
    const double idx = (n - 1.0) * 0.9;
    const size_t lo = static_cast<size_t>(std::floor(idx));
    const size_t hi = static_cast<size_t>(std::ceil(idx));
    if (lo == hi) return sorted_abs[lo];
    const double frac = idx - lo;
    return sorted_abs[lo] * (1.0 - frac) + sorted_abs[hi] * frac;
}

static std::string detectUnits(const std::vector<double>& v) {
    std::vector<double> nz;
    for (double x : v) {
        const double ax = std::fabs(x);
        if (ax > 1e-6) nz.push_back(ax);
    }
    if (nz.empty()) return "auto";
    std::sort(nz.begin(), nz.end());
    return percentile90(nz) < M_PI ? "rad" : "deg";
}

static std::vector<std::string> splitLine(const std::string& line, char delim) {
    std::vector<std::string> out;
    std::string tok;
    std::stringstream ss(line);
    while (std::getline(ss, tok, delim)) out.push_back(tok);
    return out;
}

static std::string toLower(const std::string& s) {
    std::string out = s;
    std::transform(out.begin(), out.end(), out.begin(),
                   [](unsigned char c) { return std::tolower(c); });
    return out;
}

void loadTrajectory(const std::string& path, const std::string& euler_order,
                    const std::string& euler_units, double time_shift,
                    Trajectory& tr) {
    std::ifstream f(path);
    if (!f.is_open()) throw std::runtime_error("cannot open trajectory: " + path);

    std::vector<std::string> raw;
    std::string line;
    while (std::getline(f, line)) raw.push_back(line);
    if (raw.empty()) throw std::runtime_error("empty trajectory: " + path);

    const std::string& first = raw.front();
    const long semis = static_cast<long>(
        std::count(first.begin(), first.end(), ';'));
    const long commas = static_cast<long>(
        std::count(first.begin(), first.end(), ','));
    const char delim = semis >= commas ? ';' : ',';

    std::vector<std::string> cols = splitLine(first, delim);
    std::size_t start = 0;
    bool header = false;
    std::vector<std::string> header_cols;
    if (!cols.empty()) {
        const std::string c0 = toLower(cols.front());
        std::size_t b = c0.find_first_not_of(" \t");
        std::size_t e = c0.find_last_not_of(" \t");
        const std::string cc = (b == std::string::npos)
                                   ? c0 : c0.substr(b, e - b + 1);
        if (cc == "time" || cc == "stamp" || cc == "t" || cc == "timestamp") {
            start = 1;
            header = true;
            header_cols = cols;
        }
    }

    struct Row { double t; Eigen::Vector3d p; Eigen::Quaterniond q; };
    std::vector<Row> rows;
    for (std::size_t ln = start; ln < raw.size(); ++ln) {
        std::string l = raw[ln];
        std::size_t b = l.find_first_not_of(" \t\r");
        if (b == std::string::npos) continue;                      // blank
        if (l[b] == '#') continue;                                 // comment
        std::vector<std::string> tokens = splitLine(l, delim);
        std::vector<double> v;
        v.reserve(tokens.size());
        bool ok_row = true;
        for (const std::string& tok : tokens) {
            try {
                std::size_t nxt = 0;
                v.push_back(std::stod(tok, &nxt));
            } catch (...) { ok_row = false; break; }
        }
        if (!ok_row) continue;

        Row r;
        if (v.size() >= 8 && !header) {
            // quaternion row: time,x,y,z,qx,qy,qz,qw
            r.t = v[0] + time_shift;
            r.p = Eigen::Vector3d(v[1], v[2], v[3]);
            r.q = quatNormalize(Eigen::Quaterniond(v[7], v[4], v[5], v[6]));
        } else if (v.size() >= 7) {
            double roll = v[4], pitch = v[5], yaw = v[6];
            std::string u = euler_units;
            if (u == "auto") {
                const std::string lu = detectUnits({roll, pitch, yaw});
                u = (lu != "auto") ? lu : "deg";
            }
            if (u == "deg") {
                const double k = M_PI / 180.0;
                roll *= k; pitch *= k; yaw *= k;
            } else if (u != "rad") {
                throw std::runtime_error("unknown --euler-units '" + u + "'");
            }
            r.t = v[0] + time_shift;
            r.p = Eigen::Vector3d(v[1], v[2], v[3]);
            r.q = quatFromRpy(roll, pitch, yaw, euler_order);
        } else {
            continue;
        }
        rows.push_back(r);
    }

    if (rows.size() < 2)
        throw std::runtime_error("trajectory has fewer than 2 usable rows (" +
                                 std::to_string(rows.size()) + ")");

    std::sort(rows.begin(), rows.end(),
              [](const Row& a, const Row& b) { return a.t < b.t; });

    tr.time.clear(); tr.pos.clear(); tr.quat.clear(); tr.columns.clear();
    tr.time.reserve(rows.size());
    tr.pos.reserve(rows.size());
    tr.quat.reserve(rows.size());
    for (const Row& r : rows) {
        tr.time.push_back(r.t);
        tr.pos.push_back(r.p);
        tr.quat.push_back(r.q);
    }
    tr.columns = header_cols;
    tr.minT = tr.time.front();
    tr.maxT = tr.time.back();
}