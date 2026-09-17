#include "colorise.h"
#include "parallel.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <initializer_list>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>

#include <nlohmann/json.hpp>

namespace fs = std::filesystem;

static std::string lowerStr(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(),
                   [](unsigned char c) { return std::tolower(c); });
    return s;
}

// ─────────────────────────────────────────────────────────────────────────────
// calib.json: the "camera" block (intrinsics) and the camera<->lidar extrinsic
// ─────────────────────────────────────────────────────────────────────────────
using json = nlohmann::json;

struct Block {
    std::string path;
    const json* val;
};

// Python collects every non-dict leaf of the JSON tree with its dotted path.
static void collectBlocks(const json& node, const std::string& prefix,
                          std::vector<Block>& out) {
    if (node.is_object()) {
        for (auto it = node.begin(); it != node.end(); ++it)
            collectBlocks(it.value(), prefix + it.key() + ".", out);
    } else {
        std::string p = prefix;
        if (!p.empty()) p.pop_back();          // strip the trailing '.'
        out.push_back(Block{p, &node});
    }
}

static std::vector<double> flattenNumbers(const json& v,
                                          std::vector<double>& out) {
    if (v.is_number()) out.push_back(v.get<double>());
    else if (v.is_array())
        for (const json& e : v) flattenNumbers(e, out);
    return out;
}

static std::vector<double> flattenNumbers(const json& v) {
    std::vector<double> out;
    return flattenNumbers(v, out);
}

static Eigen::Matrix4d matrixFrom16(const std::vector<double>& m) {  // row-major
    Eigen::Matrix4d T;
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c)
            T(r, c) = m[r * 4 + c];
    return T;
}

static Eigen::Matrix3d mat3From9(const std::vector<double>& v) {      // row-major
    Eigen::Matrix3d M;
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            M(r, c) = v[r * 3 + c];
    return M;
}

// Mirrors extrinsic_from_block(): flat 7/12/16 or {translation/R/t, quaternion,
// rotation, matrix} layouts.
static Eigen::Matrix4d extrinsicFromBlock(const json& b) {
    if (b.is_object()) {
        if (b.contains("translation") || b.contains("t")) {
            const json& tn = b.contains("translation") ? b["translation"] : b["t"];
            std::vector<double> tv = flattenNumbers(tn);
            if (tv.size() != 3)
                throw std::runtime_error("extrinsic: translation must be [x,y,z]");
            const Eigen::Vector3d t(tv[0], tv[1], tv[2]);

            if (b.contains("quaternion")) {
                std::vector<double> q = flattenNumbers(b["quaternion"]);
                if (q.size() == 7)
                    return makeTransform(quatNormalize(
                        Eigen::Quaterniond(q[3], q[0], q[1], q[2])), t);
                if (q.size() == 4)
                    return makeTransform(quatNormalize(
                        Eigen::Quaterniond(q[0], q[1], q[2], q[3])), t);
                throw std::runtime_error(
                    "extrinsic 'quaternion' must have 4 or 7 elements");
            }
            if (b.contains("rotation")) {
                std::vector<double> r = flattenNumbers(b["rotation"]);
                if (r.size() != 9)
                    throw std::runtime_error("extrinsic 'rotation' must be 3x3");
                return makeTransform(mat3From9(r), t);
            }
            if (b.contains("R")) {
                std::vector<double> r = flattenNumbers(b["R"]);
                if (r.size() != 9)
                    throw std::runtime_error("extrinsic 'R' must be 3x3");
                return makeTransform(mat3From9(r), t);
            }
            throw std::runtime_error("extrinsic: no rotation/quaternion");
        }
        if (b.contains("matrix")) {
            std::vector<double> m = flattenNumbers(b["matrix"]);
            if (m.size() != 16)
                throw std::runtime_error("extrinsic 'matrix' must be 4x4");
            return matrixFrom16(m);
        }
        throw std::runtime_error("extrinsic: unknown object layout");
    }

    std::vector<double> arr = flattenNumbers(b);
    if (arr.size() == 7) {
        const Eigen::Vector3d p(arr[0], arr[1], arr[2]);
        return makeTransform(quatNormalize(
            Eigen::Quaterniond(arr[6], arr[3], arr[4], arr[5])), p);
    }
    if (arr.size() == 16) return matrixFrom16(arr);
    if (arr.size() == 12) {
        std::vector<double> r9(arr.begin(), arr.begin() + 9);
        const Eigen::Vector3d t(arr[9], arr[10], arr[11]);
        return makeTransform(mat3From9(r9), t);
    }
    throw std::runtime_error("extrinsic: cannot interpret " +
                             std::to_string(arr.size()) + " numbers");
}
// Score used by the Python fallback "longest str(block) wins".
static size_t reprLength(const json& v) {
    if (v.is_number()) {
        std::ostringstream oss;
        oss << std::setprecision(17) << v.get<double>();
        return oss.str().size();
    }
    if (v.is_string()) return v.get<std::string>().size();
    if (v.is_array()) {
        size_t s = 0;
        for (const json& e : v) s += reprLength(e) + 1;
        return s;
    }
    return 1;
}

static bool isNamedKey(const std::string& path) {
    const std::size_t dot = path.rfind('.');
    const std::string last = (dot == std::string::npos)
                                 ? path : path.substr(dot + 1);
    return last == "T_camera_lidar" || last == "T_lidar_camera" ||
           last == "T_lidar_cam" || last == "T_cam_lidar";
}

static std::string stripSpaces(const std::string& s) {
    std::string out;
    out.reserve(s.size());
    for (char ch : s)
        if (ch != ' ') out.push_back(ch);
    return out;
}

// ─────────────────────────────────────────────────────────────────────────────
// Camera intrinsics (calib.json -> the "camera" block)
// ─────────────────────────────────────────────────────────────────────────────
static bool camHasIntrinsics(const json& b) {
    return b.is_object() && (b.contains("intrinsics") || b.contains("K") ||
                             b.contains("camera_matrix") || b.contains("fx") ||
                             b.contains("cam_fx"));
}

// Rank of a block path as a camera description: the plain "camera" key wins,
// then camera_*/cam* names; everything else ranks last. Ties are broken by the
// path, so the C++ (sorted map) and Python (file order) walk the same JSON to
// the same block.
static int camRank(const std::string& path) {
    const std::size_t dot = path.rfind('.');
    const std::string last =
        lowerStr(dot == std::string::npos ? path : path.substr(dot + 1));
    if (last == "camera") return 3;
    if (last.rfind("camera", 0) == 0) return 2;
    if (last.rfind("cam", 0) == 0) return 1;
    return 0;
}

// Every JSON object of the tree with its dotted path (leaves are skipped).
static void collectObjectBlocks(const json& node, const std::string& prefix,
                                std::vector<Block>& out) {
    if (!node.is_object()) return;
    std::string p = prefix;
    if (!p.empty()) p.pop_back();              // strip the trailing '.'
    if (!p.empty()) out.push_back(Block{p, &node});
    for (auto it = node.begin(); it != node.end(); ++it)
        collectObjectBlocks(it.value(), prefix + it.key() + ".", out);
}

static bool jsonNumberByKeys(const json& obj,
                             std::initializer_list<const char*> keys, double& out) {
    for (const char* k : keys)
        if (obj.contains(k) && obj[k].is_number()) {
            out = obj[k].get<double>();
            return true;
        }
    return false;
}

static std::string jsonStringByKeys(const json& obj,
                                    std::initializer_list<const char*> keys,
                                    const std::string& fallback) {
    for (const char* k : keys)
        if (obj.contains(k) && obj[k].is_string()) return obj[k].get<std::string>();
    return fallback;
}

// Camera parameters come from the camera block of calib.json — the same file
// that carries the extrinsic, so no separate camera.yaml is read:
//
//   {"camera": {"camera_model": "plumb_bob", "width": 1600, "height": 1300,
//               "intrinsics": [fx,fy,cx,cy], "distortion_coeffs": [k1,k2,p1,p2,k3]},
//    "results": {"T_lidar_camera": [...]}}
//
// key_used is the dotted path of the block that was read (usually "camera").
void loadCameraFromCalib(const std::string& path, CameraParams& cam,
                         std::string& key_used) {
    std::ifstream f(path);
    if (!f.is_open()) throw std::runtime_error("cannot open calib: " + path);
    std::stringstream ss;
    ss << f.rdbuf();
    const std::string text = ss.str();
    if (text.empty()) throw std::runtime_error("empty calib file: " + path);
    const json data = json::parse(text);

    std::vector<Block> blocks;
    collectObjectBlocks(data, "", blocks);

    std::vector<Block> cand;
    for (const Block& b : blocks)
        if (camHasIntrinsics(*b.val)) cand.push_back(b);
    if (cand.empty())
        throw std::runtime_error(
            "no camera intrinsics block in " + path +
            " (expected {\"camera\": {\"width\": .., \"height\": .., "
            "\"intrinsics\": [fx,fy,cx,cy], \"distortion_coeffs\": [..]}}; "
            "see data/calib-pinhole-2026-09-16.json)");
    std::sort(cand.begin(), cand.end(), [](const Block& a, const Block& b) {
        const int ra = camRank(a.path), rb = camRank(b.path);
        return ra != rb ? ra > rb : a.path < b.path;
    });

    const json& b = *cand.front().val;
    key_used = cand.front().path;

    // Model: "plumb_bob"/"radtan"/"pinhole" -> pinhole, "fisheye"/"equidistant"
    // -> fisheye (the same mapping the camera.yaml loader used).
    const std::string model =
        lowerStr(jsonStringByKeys(b, {"camera_model", "model", "distortion_model",
                                      "cam_model"}, "PinholeCamera"));
    cam.model = (model.find("fish") != std::string::npos ||
                 model.find("equidistant") != std::string::npos)
                    ? "fisheye" : "pinhole";

    // Image size — required: the edge gate and the occlusion z-buffer are sized
    // from it, and camera.yaml is no longer read.
    double w = 0.0, h = 0.0;
    if (!jsonNumberByKeys(b, {"width", "image_width", "cam_width"}, w) ||
        !jsonNumberByKeys(b, {"height", "image_height", "cam_height"}, h))
        throw std::runtime_error(
            "camera block '" + key_used + "' in " + path +
            " has no image size: add \"width\" and \"height\" "
            "(see data/calib-pinhole-2026-09-16.json)");
    cam.width  = static_cast<int>(w);
    cam.height = static_cast<int>(h);

    // Intrinsics: [fx,fy,cx,cy], a 3x3 K (row-major), or the individual keys.
    if (b.contains("intrinsics")) {
        const std::vector<double> v = flattenNumbers(b["intrinsics"]);
        if (v.size() < 4)
            throw std::runtime_error("camera block '" + key_used + "' in " + path +
                                     ": 'intrinsics' must be [fx,fy,cx,cy]");
        cam.K << v[0], 0.0, v[2],
                 0.0, v[1], v[3],
                 0.0, 0.0, 1.0;
    } else if (b.contains("K") || b.contains("camera_matrix")) {
        const std::vector<double> v =
            flattenNumbers(b.contains("K") ? b["K"] : b["camera_matrix"]);
        if (v.size() != 9)
            throw std::runtime_error("camera block '" + key_used + "' in " + path +
                                     ": 'K' must be a 3x3 matrix");
        cam.K = mat3From9(v);
    } else {
        double fx = 0.0, fy = 0.0, cx = 0.0, cy = 0.0;
        if (!jsonNumberByKeys(b, {"fx", "cam_fx"}, fx) ||
            !jsonNumberByKeys(b, {"fy", "cam_fy"}, fy) ||
            !jsonNumberByKeys(b, {"cx", "cam_cx"}, cx) ||
            !jsonNumberByKeys(b, {"cy", "cam_cy"}, cy))
            throw std::runtime_error(
                "camera block '" + key_used + "' in " + path +
                ": no intrinsics (expected \"intrinsics\": [fx,fy,cx,cy])");
        cam.K << fx, 0.0, cx,
                 0.0, fy, cy,
                 0.0, 0.0, 1.0;
    }

    // Distortion: passed to OpenCV as-is, whatever its length.
    if (b.contains("distortion_coeffs")) cam.dist = flattenNumbers(b["distortion_coeffs"]);
    else if (b.contains("dist_coeffs"))  cam.dist = flattenNumbers(b["dist_coeffs"]);
    else if (b.contains("D"))            cam.dist = flattenNumbers(b["D"]);
    else {
        cam.dist.clear();
        for (int d = 0; d <= 5; ++d) {
            const std::string k = "cam_d" + std::to_string(d);
            if (b.contains(k) && b[k].is_number())
                cam.dist.push_back(b[k].get<double>());
        }
    }
}

void loadCalib(const std::string& path, const std::string& name,
               const std::string& direction, Eigen::Matrix4d& T_cam_lidar,
               std::string& key_used, std::string& direction_used) {
    std::ifstream f(path);
    if (!f.is_open()) throw std::runtime_error("cannot open calib: " + path);
    std::stringstream ss;
    ss << f.rdbuf();
    std::string text = ss.str();
    if (text.empty()) throw std::runtime_error("empty calib file: " + path);
    const json data = json::parse(text);

    std::vector<Block> blocks;
    collectBlocks(data, "", blocks);

    std::vector<Block> named;
    for (const Block& b : blocks)
        if (isNamedKey(b.path)) named.push_back(b);

    Block found;
    std::string full;
    if (!name.empty()) {
        bool ok = false;
        for (const Block& b : blocks) {
            if (b.path == name ||
                (b.path.size() >= name.size() &&
                 b.path.rfind(name) == b.path.size() - name.size())) {
                found = b;
                full = name;
                ok = true;
                break;
            }
        }
        if (!ok)
            throw std::runtime_error("extrinsic key '" + name +
                                     "' not found in " + path);
    } else if (!named.empty()) {
        found = named.front();
        full = found.path;
    } else {
        std::vector<Block> cand;
        for (const Block& b : blocks) {
            bool arr_ok = false;
            if (b.val->is_array() || b.val->is_number()) {
                const std::vector<double> a = flattenNumbers(*b.val);
                arr_ok = (a.size() == 4 || a.size() == 7 ||
                          a.size() == 12 || a.size() == 16);
            }
            if (b.val->is_object() || arr_ok) cand.push_back(b);
        }
        if (cand.empty())
            throw std::runtime_error("no extrinsic-like block found in " + path);
        Block best = cand.front();
        size_t best_len = 0;
        for (const Block& b : cand) {
            const size_t l = reprLength(*b.val);
            if (l > best_len) { best_len = l; best = b; }
        }
        found = best;
        full = best.path;
    }

    Eigen::Matrix4d T = extrinsicFromBlock(*found.val);

    const std::string lower_key = stripSpaces(lowerStr(full));
    const bool lidar_from_cam =
        lower_key.find("lidar_camera") != std::string::npos ||
        lower_key.find("lidar_from_cam") != std::string::npos;
    if (lidar_from_cam) T = invertTransform(T);

    if (direction == "lidar_from_camera") T = invertTransform(T);

    T_cam_lidar = T;
    key_used = full;
    direction_used = direction.empty() ? "auto" : direction;
}
// ─────────────────────────────────────────────────────────────────────────────
// Point cloud (binary PCD)
// ─────────────────────────────────────────────────────────────────────────────
struct PcdHeader {
    std::vector<std::string> fields;
    std::vector<int> sizes;
    std::vector<char> types;
    long long npts = 0;
    std::string data_kind;
    std::streamoff data_offset = 0;
};

static std::vector<std::string> splitWs(const std::string& line) {
    std::istringstream ss(line);
    std::vector<std::string> out;
    std::string tok;
    while (ss >> tok) out.push_back(tok);
    return out;
}

static PcdHeader readPcdHeader(std::ifstream& f) {
    PcdHeader hdr;
    std::string line;
    while (std::getline(f, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        std::size_t b = line.find_first_not_of(" \t");
        if (b == std::string::npos) continue;
        const std::string trimmed = line.substr(b);
        if (trimmed[0] == '#') continue;
        std::vector<std::string> parts = splitWs(trimmed);
        if (parts.empty()) continue;
        if (parts[0] == "FIELDS") hdr.fields.assign(parts.begin() + 1, parts.end());
        else if (parts[0] == "SIZE")
            for (std::size_t i = 1; i < parts.size(); ++i)
                hdr.sizes.push_back(std::atoi(parts[i].c_str()));
        else if (parts[0] == "TYPE" && parts.size() >= 2)
            for (std::size_t i = 1; i < parts.size() && i - 1 < hdr.sizes.size(); ++i)
                hdr.types.push_back(parts[i][0]);
        else if (parts[0] == "POINTS") hdr.npts = std::atoll(parts[1].c_str());
        else if (parts[0] == "DATA") {
            hdr.data_kind = (parts.size() > 1) ? parts[1] : "binary";
            hdr.data_offset = f.tellg();
            break;
        }
    }
    return hdr;
}

// ─────────────────────────────────────────────────────────────────────────────
// LAS (.las) — uncompressed LAS 1.0-1.4, point formats 0-10.
//
// C++ replacement of the optional laspy path in colorise_offline.py; no
// external library. Every LAS point record starts with int32 X, Y, Z (scaled
// by the header scale factors and shifted by the header offsets) followed by
// uint16 intensity — that layout holds for all point formats 0-10, so the
// reader does not need per-format tables. Compressed .laz is rejected, same
// as the Python script without the laszip backend.
// ─────────────────────────────────────────────────────────────────────────────
static uint32_t rdU16(const unsigned char* p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8);
}

static uint32_t rdU32(const unsigned char* p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static int32_t rdI32(const unsigned char* p) {
    return (int32_t)rdU32(p);
}

static uint64_t rdU64(const unsigned char* p) {
    uint64_t v = 0;
    for (int i = 7; i >= 0; --i) v = (v << 8) | (uint64_t)p[i];
    return v;
}

// LAS stores doubles little-endian; assemble the bit pattern from bytes so
// this is exact on any little-endian host.
static double rdF64(const unsigned char* p) {
    const uint64_t v = rdU64(p);
    double d;
    memcpy(&d, &v, sizeof(d));
    return d;
}

static void loadLas(const std::string& path, Cloud& cloud, int jobs) {
    std::ifstream f(path, std::ios::binary);
    if (!f.is_open()) throw std::runtime_error("cannot open cloud: " + path);

    // Public header block: 227 bytes (LAS 1.0-1.2), 235 (1.3), 375 (1.4).
    unsigned char h[375];
    std::memset(h, 0, sizeof(h));
    f.read(reinterpret_cast<char*>(h), sizeof(h));
    if (f.gcount() < 227)
        throw std::runtime_error("not a LAS file (too small): " + path);
    if (std::memcmp(h, "LASF", 4) != 0)
        throw std::runtime_error("not a LAS file (bad signature): " + path);

    // Canonical public-header-block offsets (identical in LAS 1.0-1.4; the
    // 1.3/1.4 additions start at 227). Verified against laspy-written files.
    const int header_size   = static_cast<int>(rdU16(h + 94));
    const uint32_t pts_off  = rdU32(h + 96);      // offset to point data
    const uint16_t rec_len  = rdU16(h + 105);     // bytes per point record
    const uint32_t n_legacy = rdU32(h + 107);
    const double sx = rdF64(h + 131), sy = rdF64(h + 139), sz = rdF64(h + 147);
    const double ox = rdF64(h + 155), oy = rdF64(h + 163), oz = rdF64(h + 171);
    const unsigned char fmt_byte = h[104];

    if (header_size < 227)
        throw std::runtime_error("bad LAS header size " +
                                 std::to_string(header_size) + ": " + path);
    if ((fmt_byte & 0x20) != 0)
        throw std::runtime_error(
            "compressed .laz input is not supported; decompress with laszip "
            "first: " + path);
    const int pfmt = static_cast<int>(fmt_byte & 0x0F);
    if (pfmt > 10)
        throw std::runtime_error("unsupported LAS point format " +
                                 std::to_string(pfmt) + ": " + path);

    long long npts = static_cast<long long>(n_legacy);
    if (npts <= 0 && header_size >= 255) {
        // LAS 1.4 (375-byte header): the legacy count is 0 once the point
        // count exceeds 2^32-1; the extended count lives at offset 247.
        const uint64_t ext = rdU64(h + 247);
        if (ext > 0) npts = static_cast<long long>(ext);
    }
    if (npts <= 0)
        throw std::runtime_error("LAS header reports no points: " + path);
    if (rec_len < 14)
        throw std::runtime_error("bad LAS point data record length " +
                                 std::to_string(rec_len) + ": " + path);
    if (pts_off == 0)
        throw std::runtime_error("LAS header has no point data offset: " + path);

    f.clear();
    f.seekg(static_cast<std::streamoff>(pts_off));
    std::vector<unsigned char> buf(static_cast<std::size_t>(rec_len) *
                                   static_cast<std::size_t>(npts));
    f.read(reinterpret_cast<char*>(buf.data()),
           static_cast<std::streamsize>(buf.size()));
    if (static_cast<std::streamsize>(buf.size()) != f.gcount())
        throw std::runtime_error("truncated LAS point data: " + path);

    cloud.x.resize(static_cast<std::size_t>(npts));
    cloud.y.resize(static_cast<std::size_t>(npts));
    cloud.z.resize(static_cast<std::size_t>(npts));
    cloud.intensity.resize(static_cast<std::size_t>(npts));  // present in all formats

    double* xp = cloud.x.data();
    double* yp = cloud.y.data();
    double* zp = cloud.z.data();
    float*  ip = cloud.intensity.data();
    const unsigned char* bp = buf.data();
    const std::size_t rl = static_cast<std::size_t>(rec_len);
    const std::size_t nn = static_cast<std::size_t>(npts);
    parallelForRange(nn, jobs, [&](std::size_t lo, std::size_t hi) {
        for (std::size_t i = lo; i < hi; ++i) {
            const unsigned char* p = bp + i * rl;
            xp[i] = sx * static_cast<double>(rdI32(p))      + ox;
            yp[i] = sy * static_cast<double>(rdI32(p + 4))  + oy;
            zp[i] = sz * static_cast<double>(rdI32(p + 8))  + oz;
            ip[i] = static_cast<float>(rdU16(p + 12));
        }
    });
}

void loadCloud(const std::string& path, Cloud& cloud, int jobs) {
    std::size_t dot = path.rfind('.');
    std::string ext = dot == std::string::npos
                          ? "" : lowerStr(path.substr(dot + 1));
    if (ext == "las") {
        loadLas(path, cloud, jobs);
        return;
    }
    if (ext != "pcd")
        throw std::runtime_error("unsupported cloud: ." + ext +
                                 " (only binary .pcd and .las input is supported)");

    std::ifstream f(path, std::ios::binary);
    if (!f.is_open()) throw std::runtime_error("cannot open cloud: " + path);
    PcdHeader hdr = readPcdHeader(f);
    if (hdr.data_kind != "binary")
        throw std::runtime_error("unsupported PCD data kind: " + hdr.data_kind);

    const int nf = static_cast<int>(hdr.fields.size());
    if (nf == 0 || hdr.npts < 0)
        throw std::runtime_error("bad PCD header in: " + path);
    while (static_cast<int>(hdr.sizes.size()) < nf) hdr.sizes.push_back(4);
    while (static_cast<int>(hdr.types.size()) < nf) hdr.types.push_back('F');

    std::vector<int> off(nf, 0);
    for (int i = 1; i < nf; ++i) off[i] = off[i - 1] + hdr.sizes[i - 1];
    const long long row = off.back() + hdr.sizes.back();

    std::vector<char> buf(static_cast<size_t>(row) * static_cast<size_t>(hdr.npts));
    f.seekg(hdr.data_offset);
    f.read(buf.data(), static_cast<std::streamsize>(buf.size()));

    int ix = -1, iy = -1, iz = -1, ii = -1;
    for (int i = 0; i < nf; ++i) {
        if (hdr.fields[i] == "x") ix = i;
        else if (hdr.fields[i] == "y") iy = i;
        else if (hdr.fields[i] == "z") iz = i;
        else if (hdr.fields[i] == "intensity") ii = i;
    }
    if (ix < 0 || iy < 0 || iz < 0)
        throw std::runtime_error("PCD must have x y z fields: " + path);

    auto readVal = [&](const char* p, int field) -> double {
        const char t = hdr.types[field];
        const int s = hdr.sizes[field];
        if (t == 'F' && s == 4) { float v; memcpy(&v, p, 4); return v; }
        if (t == 'F' && s == 8) { double v; memcpy(&v, p, 8); return v; }
        if (t == 'U' && s == 4) { uint32_t v; memcpy(&v, p, 4); return v; }
        if (t == 'U' && s == 8) { uint64_t v; memcpy(&v, p, 8); return v; }
        if (t == 'I' && s == 4) { int32_t v; memcpy(&v, p, 4); return v; }
        if (t == 'I' && s == 8) { int64_t v; memcpy(&v, p, 8); return v; }
        throw std::runtime_error("unsupported PCD field type/size");
    };

    // Validate every field type/size up front: the parallel fill below runs
    // in worker threads where an escaping exception would abort the process.
    for (int fld = 0; fld < nf; ++fld) {
        const char t = hdr.types[fld];
        const int s2 = hdr.sizes[fld];
        const bool ok = (t == 'F' && (s2 == 4 || s2 == 8)) ||
                        (t == 'U' && (s2 == 4 || s2 == 8)) ||
                        (t == 'I' && (s2 == 4 || s2 == 8));
        if (!ok)
            throw std::runtime_error("unsupported PCD field type/size for field '" +
                                     hdr.fields[fld] + "'");
    }

    // Field values are decoded in parallel, by index (same order/bytes as the
    // sequential push_back version).
    const std::size_t nn = static_cast<std::size_t>(hdr.npts);
    cloud.x.resize(nn); cloud.y.resize(nn); cloud.z.resize(nn);
    if (ii >= 0) cloud.intensity.resize(nn); else cloud.intensity.clear();

    double* xp = cloud.x.data();
    double* yp = cloud.y.data();
    double* zp = cloud.z.data();
    float*  ip = (ii >= 0) ? cloud.intensity.data() : nullptr;
    parallelForRange(nn, jobs, [&](std::size_t lo, std::size_t hi) {
        for (std::size_t i = lo; i < hi; ++i) {
            const char* base = buf.data() + i * static_cast<std::size_t>(row);
            xp[i] = readVal(base + off[ix], ix);
            yp[i] = readVal(base + off[iy], iy);
            zp[i] = readVal(base + off[iz], iz);
            if (ip) ip[i] = static_cast<float>(readVal(base + off[ii], ii));
        }
    });
}

// LAS output — a C++ addition (the Python script only ever writes PCD).
//
// Uncompressed LAS 1.2, point data record format 3 (X Y Z + intensity + GPS
// time + RGB), 34 bytes per record. Coordinates are quantised with a 1 mm
// scale and a per-axis offset at the data minimum; 8-bit RGB is expanded to
// the 16-bit LAS fields as v*257, so viewers that take the visible colour
// from the high byte show the same colours as the PCD output does.
// ─────────────────────────────────────────────────────────────────────────────
static void saveLas(const std::string& path, const Cloud& cloud,
                    const std::vector<char>& keep,
                    const std::vector<uint32_t>& rgb, int jobs) {
    std::vector<int> sel;
    sel.reserve(cloud.n());
    for (std::size_t i = 0; i < cloud.n(); ++i)
        if (keep[i]) sel.push_back(static_cast<int>(i));
    const std::size_t n = sel.size();
    if (static_cast<double>(n) > 4294967000.0)
        throw std::runtime_error("too many points for LAS 1.2 output: " + path);

    // Bounding box of the points that are actually written (zeros when the
    // selection is empty, matching the empty-PCD behaviour of the PCD path).
    double mn[3] = { 0.0, 0.0, 0.0 };
    double mx[3] = { 0.0, 0.0, 0.0 };
    if (n > 0) {
        mn[0] = mn[1] = mn[2] = std::numeric_limits<double>::infinity();
        mx[0] = mx[1] = mx[2] = -std::numeric_limits<double>::infinity();
        for (int si : sel) {
            const double v[3] = { cloud.x[si], cloud.y[si], cloud.z[si] };
            for (int a = 0; a < 3; ++a) {
                if (v[a] < mn[a]) mn[a] = v[a];
                if (v[a] > mx[a]) mx[a] = v[a];
            }
        }
    }

    double scale = 0.001;                            // 1 mm, lidar convention
    double span = 0.0;
    for (int a = 0; a < 3; ++a) span = std::max(span, mx[a] - mn[a]);
    if (span > 0.0 && span / scale > 2147483000.0)   // keep raw int32-safe
        scale = span / 2147483000.0;
    double off[3];
    for (int a = 0; a < 3; ++a) off[a] = std::floor(mn[a]);

    auto put16 = [](unsigned char* p, uint32_t v) {
        p[0] = static_cast<unsigned char>(v & 0xFF);
        p[1] = static_cast<unsigned char>((v >> 8) & 0xFF);
    };
    auto put32 = [](unsigned char* p, uint32_t v) {
        p[0] = static_cast<unsigned char>(v & 0xFF);
        p[1] = static_cast<unsigned char>((v >> 8) & 0xFF);
        p[2] = static_cast<unsigned char>((v >> 16) & 0xFF);
        p[3] = static_cast<unsigned char>((v >> 24) & 0xFF);
    };
    auto putf64 = [](unsigned char* p, double d) {
        uint64_t bits;
        std::memcpy(&bits, &d, sizeof(bits));
        for (int i = 0; i < 8; ++i)
            p[i] = static_cast<unsigned char>((bits >> (8 * i)) & 0xFF);
    };

    // Public header block, LAS 1.2 (227 bytes).
    std::vector<unsigned char> hdr(227, 0);
    std::memcpy(hdr.data(), "LASF", 4);
    hdr[24] = 1;                                     // version major
    hdr[25] = 2;                                     // version minor
    const std::string sysid = "PointCloudColorising";
    const std::string gensw = "colorise_offline";
    for (std::size_t i = 0; i < sysid.size() && i < 32; ++i) hdr[26 + i] = (unsigned char)sysid[i];
    for (std::size_t i = 0; i < gensw.size() && i < 32; ++i) hdr[58 + i] = (unsigned char)gensw[i];
    const std::time_t now = std::time(nullptr);
    if (const std::tm* tm = std::localtime(&now)) {
        put16(hdr.data() + 90, static_cast<uint32_t>(tm->tm_yday + 1));
        put16(hdr.data() + 92, static_cast<uint32_t>(tm->tm_year + 1900));
    }
    put16(hdr.data() + 94, 227);                     // header size
    put32(hdr.data() + 96, 227);                     // offset to point data
    put32(hdr.data() + 100, 0);                      // number of VLRs
    hdr[104] = 3;                                    // point data format 3
    put16(hdr.data() + 105, 34);                     // record length
    put32(hdr.data() + 107, static_cast<uint32_t>(n));   // legacy point count
    put32(hdr.data() + 111, static_cast<uint32_t>(n));   // points by return 1
    putf64(hdr.data() + 131, scale);
    putf64(hdr.data() + 139, scale);
    putf64(hdr.data() + 147, scale);
    putf64(hdr.data() + 155, off[0]);
    putf64(hdr.data() + 163, off[1]);
    putf64(hdr.data() + 171, off[2]);
    // Header bbox = the quantised data bbox, so the header matches the records
    // exactly (rounding is monotonic, so the extremes map through directly).
    auto quant = [&](double v, int a) {
        return static_cast<double>(static_cast<int32_t>(
                   std::nearbyint((v - off[a]) / scale))) * scale + off[a];
    };
    putf64(hdr.data() + 179, quant(mx[0], 0)); putf64(hdr.data() + 187, quant(mn[0], 0));
    putf64(hdr.data() + 195, quant(mx[1], 1)); putf64(hdr.data() + 203, quant(mn[1], 1));
    putf64(hdr.data() + 211, quant(mx[2], 2)); putf64(hdr.data() + 219, quant(mn[2], 2));

    // Point records (34 bytes each): encoded in parallel, written in one go.
    const std::size_t rec = 34;
    std::vector<unsigned char> buf(n * rec);
    unsigned char* bp = buf.data();
    parallelForRange(n, jobs, [&](std::size_t lo, std::size_t hi) {
        for (std::size_t k = lo; k < hi; ++k) {
            const int si = sel[k];
            unsigned char* p = bp + k * rec;
            put32(p + 0, static_cast<uint32_t>(static_cast<int32_t>(
                             std::nearbyint((cloud.x[si] - off[0]) / scale))));
            put32(p + 4, static_cast<uint32_t>(static_cast<int32_t>(
                             std::nearbyint((cloud.y[si] - off[1]) / scale))));
            put32(p + 8, static_cast<uint32_t>(static_cast<int32_t>(
                             std::nearbyint((cloud.z[si] - off[2]) / scale))));
            // Round half-to-even (std::nearbyint), the convention used
            // everywhere else in this tool; non-finite intensity maps to 0.
            const double raw_i = cloud.hasIntensity()
                                     ? static_cast<double>(cloud.intensity[si])
                                     : 0.0;
            long long iv = std::isfinite(raw_i)
                               ? static_cast<long long>(std::nearbyint(raw_i))
                               : 0;
            if (iv < 0) iv = 0;
            if (iv > 65535) iv = 65535;
            put16(p + 12, static_cast<uint32_t>(iv));
            p[14] = 0x09;                            // return 1 of 1
            p[15] = 0;                               // never classified
            p[16] = 0;                               // scan angle rank
            p[17] = 0;                               // user data
            put16(p + 18, 0);                        // point source id
            putf64(p + 20, 0.0);                     // GPS time
            const uint32_t c = rgb[si];
            put16(p + 28, static_cast<uint32_t>(((c >> 16) & 0xFF) * 257));
            put16(p + 30, static_cast<uint32_t>(((c >> 8) & 0xFF) * 257));
            put16(p + 32, static_cast<uint32_t>((c & 0xFF) * 257));
        }
    });

    std::ofstream f(path, std::ios::binary);
    if (!f.is_open()) throw std::runtime_error("cannot write output: " + path);
    f.write(reinterpret_cast<const char*>(hdr.data()),
            static_cast<std::streamsize>(hdr.size()));
    f.write(reinterpret_cast<const char*>(buf.data()),
            static_cast<std::streamsize>(buf.size()));
}

// Writes the selected subset as a binary XYZI+RGB PCD (same bytes as Python),
// or as LAS when the path ends in .las (see saveLas above).
void saveCloud(const std::string& path, const Cloud& cloud,
               const std::vector<char>& keep, const std::vector<uint32_t>& rgb,
               int jobs) {
    const std::size_t odot = path.rfind('.');
    const std::string oext =
        odot == std::string::npos ? "" : lowerStr(path.substr(odot + 1));
    if (oext == "las") {
        saveLas(path, cloud, keep, rgb, jobs);
        return;
    }
    if (oext == "laz")
        throw std::runtime_error(
            "compressed .laz output is not supported; use .las or .pcd: " + path);

    std::vector<int> sel;
    sel.reserve(cloud.n());
    for (size_t i = 0; i < cloud.n(); ++i)
        if (keep[i]) sel.push_back(static_cast<int>(i));
    const long long n = static_cast<long long>(sel.size());

    std::ofstream f(path, std::ios::binary);
    if (!f.is_open()) throw std::runtime_error("cannot write output: " + path);
    std::ostringstream hdr;
    hdr << "# .PCD v0.7 - Point Cloud Data file format\n"
        << "VERSION 0.7\n"
        << "FIELDS x y z intensity rgb\n"
        << "SIZE 4 4 4 4 4\n"
        << "TYPE F F F F U\n"
        << "COUNT 1 1 1 1 1\n"
        << "WIDTH " << n << "\nHEIGHT 1\nPOINTS " << n << "\nDATA binary\n";
    const std::string hs = hdr.str();
    f.write(hs.data(), static_cast<std::streamsize>(hs.size()));

    std::vector<char> row(5 * 4);
    for (int si : sel) {
        float v;
        v = static_cast<float>(cloud.x[si]); memcpy(row.data() + 0, &v, 4);
        v = static_cast<float>(cloud.y[si]); memcpy(row.data() + 4, &v, 4);
        v = static_cast<float>(cloud.z[si]); memcpy(row.data() + 8, &v, 4);
        v = cloud.hasIntensity() ? cloud.intensity[si] : 0.0f;
        memcpy(row.data() + 12, &v, 4);
        const uint32_t c = rgb[si];
        memcpy(row.data() + 16, &c, 4);
        f.write(row.data(), static_cast<std::streamsize>(row.size()));
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// LAS with timestamps (LasRaw, cube colourisation mode)
// ─────────────────────────────────────────────────────────────────────────────
static void wrU16(unsigned char* p, uint32_t v) {
    p[0] = static_cast<unsigned char>(v & 0xFF);
    p[1] = static_cast<unsigned char>((v >> 8) & 0xFF);
}

static void wrU32(unsigned char* p, uint32_t v) {
    p[0] = static_cast<unsigned char>(v & 0xFF);
    p[1] = static_cast<unsigned char>((v >> 8) & 0xFF);
    p[2] = static_cast<unsigned char>((v >> 16) & 0xFF);
    p[3] = static_cast<unsigned char>((v >> 24) & 0xFF);
}

static void wrU64(unsigned char* p, uint64_t v) {
    for (int i = 0; i < 8; ++i)
        p[i] = static_cast<unsigned char>((v >> (8 * i)) & 0xFF);
}

static void wrF64(unsigned char* p, double d) {
    uint64_t bits;
    std::memcpy(&bits, &d, sizeof(bits));
    for (int i = 0; i < 8; ++i)
        p[i] = static_cast<unsigned char>((bits >> (8 * i)) & 0xFF);
}

// gps_time byte offset inside a point record; -1 for formats without it.
static int lasTimeOffset(int fmt) {
    if (fmt == 0 || fmt == 2) return -1;
    return (fmt >= 6) ? 22 : 20;
}

void loadLasRaw(const std::string& path, LasRaw& las) {
    std::ifstream f(path, std::ios::binary);
    if (!f.is_open()) throw std::runtime_error("cannot open cloud: " + path);
    unsigned char h[375];
    std::memset(h, 0, sizeof(h));
    f.read(reinterpret_cast<char*>(h), sizeof(h));
    if (f.gcount() < 227)
        throw std::runtime_error("not a LAS file (too small): " + path);
    if (std::memcmp(h, "LASF", 4) != 0)
        throw std::runtime_error("not a LAS file (bad signature): " + path);
    if ((h[104] & 0x20) != 0)
        throw std::runtime_error("compressed .laz input is not supported; "
                                 "decompress with laszip first: " + path);

    const int      header_size = static_cast<int>(rdU16(h + 94));
    const uint32_t pts_off     = rdU32(h + 96);
    const uint16_t rec_len     = rdU16(h + 105);
    const uint32_t n_legacy    = rdU32(h + 107);
    const int      fmt         = static_cast<int>(h[104] & 0x0F);
    if (fmt > 10)
        throw std::runtime_error("unsupported LAS point format " +
                                 std::to_string(fmt) + ": " + path);
    if (rec_len < 20)
        throw std::runtime_error("bad LAS point data record length " +
                                 std::to_string(rec_len) + ": " + path);
    if (header_size < 227 || pts_off == 0)
        throw std::runtime_error("bad LAS header: " + path);

    long long npts = static_cast<long long>(n_legacy);
    if (npts <= 0 && header_size >= 255) {
        const uint64_t ext = rdU64(h + 247);
        if (ext > 0) npts = static_cast<long long>(ext);
    }
    if (npts <= 0) throw std::runtime_error("LAS header reports no points: " + path);

    las.fmt     = fmt;
    las.rec_len = static_cast<int>(rec_len);
    las.sx = rdF64(h + 131); las.sy = rdF64(h + 139); las.sz = rdF64(h + 147);
    las.ox = rdF64(h + 155); las.oy = rdF64(h + 163); las.oz = rdF64(h + 171);

    const std::size_t n = static_cast<std::size_t>(npts);
    las.X.resize(n);
    las.Y.resize(n);
    las.Z.resize(n);
    las.intensity.resize(n);
    const int t_off = lasTimeOffset(fmt);
    las.time.clear();
    if (t_off > 0 && rec_len >= t_off + 8) las.time.resize(n);

    f.clear();
    f.seekg(static_cast<std::streamoff>(pts_off));
    const long long chunk = 1 << 20;
    std::vector<unsigned char> buf(static_cast<std::size_t>(rec_len) *
                                   static_cast<std::size_t>(chunk));
    long long done = 0;
    while (done < npts) {
        const long long want = std::min<long long>(chunk, npts - done);
        f.read(reinterpret_cast<char*>(buf.data()),
               static_cast<std::streamsize>(rec_len) * want);
        const long long have = static_cast<long long>(f.gcount()) / rec_len;
        if (have <= 0)
            throw std::runtime_error("truncated LAS point data: " + path);
        const unsigned char* q = buf.data();
        for (long long k = 0; k < have; ++k, q += rec_len) {
            const std::size_t i = static_cast<std::size_t>(done + k);
            las.X[i] = rdI32(q + 0);
            las.Y[i] = rdI32(q + 4);
            las.Z[i] = rdI32(q + 8);
            las.intensity[i] = static_cast<uint16_t>(rdU16(q + 12));
            if (!las.time.empty()) las.time[i] = rdF64(q + t_off);
        }
        done += have;
    }
}

void saveLas7(const std::string& path, const LasRaw& las,
              const std::vector<uint32_t>& order, const std::vector<char>& keep,
              const std::vector<uint32_t>& rgb) {
    std::vector<int> sel;
    sel.reserve(order.size());
    for (uint32_t i : order)
        if (keep[static_cast<std::size_t>(i)]) sel.push_back(static_cast<int>(i));
    const std::size_t n = sel.size();

    int32_t bx[6] = { 0, 0, 0, 0, 0, 0 };
    if (n > 0) {
        const int first = sel.front();
        bx[0] = bx[1] = las.X[static_cast<std::size_t>(first)];
        bx[2] = bx[3] = las.Y[static_cast<std::size_t>(first)];
        bx[4] = bx[5] = las.Z[static_cast<std::size_t>(first)];
        for (int i : sel) {
            bx[0] = std::min(bx[0], las.X[static_cast<std::size_t>(i)]);
            bx[1] = std::max(bx[1], las.X[static_cast<std::size_t>(i)]);
            bx[2] = std::min(bx[2], las.Y[static_cast<std::size_t>(i)]);
            bx[3] = std::max(bx[3], las.Y[static_cast<std::size_t>(i)]);
            bx[4] = std::min(bx[4], las.Z[static_cast<std::size_t>(i)]);
            bx[5] = std::max(bx[5], las.Z[static_cast<std::size_t>(i)]);
        }
    }

    // LAS 1.4 public header block (375 bytes), point format 7 (RGB + gps_time).
    std::vector<unsigned char> hdr(375, 0);
    std::memcpy(hdr.data(), "LASF", 4);
    hdr[24] = 1;                                     // version major
    hdr[25] = 4;                                     // version minor
    const std::string sysid = "PointCloudColorising";
    const std::string gensw = "colorise_map (cube mode)";
    for (std::size_t i = 0; i < sysid.size() && i < 32; ++i) hdr[26 + i] = (unsigned char)sysid[i];
    for (std::size_t i = 0; i < gensw.size() && i < 32; ++i) hdr[58 + i] = (unsigned char)gensw[i];
    const std::time_t now = std::time(nullptr);
    if (const std::tm* tm = std::localtime(&now)) {
        wrU16(hdr.data() + 90, static_cast<uint32_t>(tm->tm_yday + 1));
        wrU16(hdr.data() + 92, static_cast<uint32_t>(tm->tm_year + 1900));
    }
    wrU16(hdr.data() + 94, 375);                     // header size
    wrU32(hdr.data() + 96, 375);                     // offset to point data
    wrU32(hdr.data() + 100, 0);                      // number of VLRs
    hdr[104] = 7;                                    // point data format 7
    wrU16(hdr.data() + 105, 36);                     // record length
    const uint32_t legacy = (n <= 0xFFFFFFFFu) ? static_cast<uint32_t>(n) : 0u;
    wrU32(hdr.data() + 107, legacy);                 // legacy point count
    wrU32(hdr.data() + 111, legacy);                 // legacy points by return 1
    wrF64(hdr.data() + 131, las.sx); wrF64(hdr.data() + 139, las.sy); wrF64(hdr.data() + 147, las.sz);
    wrF64(hdr.data() + 155, las.ox); wrF64(hdr.data() + 163, las.oy); wrF64(hdr.data() + 171, las.oz);
    // Header bbox: the quantised data bbox, exactly like the LAS 1.2 writer.
    auto coord = [&](int32_t v, int a) {
        const double s = (a == 0) ? las.sx : (a == 1) ? las.sy : las.sz;
        const double o = (a == 0) ? las.ox : (a == 1) ? las.oy : las.oz;
        return s * static_cast<double>(v) + o;
    };
    wrF64(hdr.data() + 179, coord(bx[1], 0)); wrF64(hdr.data() + 187, coord(bx[0], 0));
    wrF64(hdr.data() + 195, coord(bx[3], 1)); wrF64(hdr.data() + 203, coord(bx[2], 1));
    wrF64(hdr.data() + 211, coord(bx[5], 2)); wrF64(hdr.data() + 219, coord(bx[4], 2));
    wrU64(hdr.data() + 227, 0);                      // waveform data start
    wrU64(hdr.data() + 235, 0);                      // first EVLR
    wrU32(hdr.data() + 243, 0);                      // number of EVLRs
    {
        uint64_t bits = static_cast<uint64_t>(n);
        for (int i = 0; i < 8; ++i)
            hdr[247 + i] = static_cast<unsigned char>((bits >> (8 * i)) & 0xFF);
    }
    for (int i = 0; i < 8; ++i) hdr[255 + i] = hdr[247 + i];   // points by return 1

    std::ofstream f(path, std::ios::binary);
    if (!f.is_open()) throw std::runtime_error("cannot write output: " + path);
    f.write(reinterpret_cast<const char*>(hdr.data()),
            static_cast<std::streamsize>(hdr.size()));

    const std::size_t rec = 36;
    std::vector<unsigned char> buf(rec);
    for (int i : sel) {
        const std::size_t k = static_cast<std::size_t>(i);
        wrU32(buf.data() + 0, static_cast<uint32_t>(las.X[k]));
        wrU32(buf.data() + 4, static_cast<uint32_t>(las.Y[k]));
        wrU32(buf.data() + 8, static_cast<uint32_t>(las.Z[k]));
        wrU16(buf.data() + 12, las.intensity[k]);
        wrU16(buf.data() + 14, 0x11);                // return 1 of 1
        buf[16] = 0;                                 // classification
        buf[17] = 0;                                 // user data
        wrU16(buf.data() + 18, 0);                   // scan angle
        wrU16(buf.data() + 20, 0);                   // point source id
        wrF64(buf.data() + 22, las.time.empty() ? 0.0 : las.time[k]);
        const uint32_t c = rgb[k];
        const uint32_t r = (c >> 16) & 0xFF, g = (c >> 8) & 0xFF, b = c & 0xFF;
        // RGB занимает байты 30..35 (после gps_time): 8 бит -> 16 бит (x257)
        wrU16(buf.data() + 30, (r << 8) | r);
        wrU16(buf.data() + 32, (g << 8) | g);
        wrU16(buf.data() + 34, (b << 8) | b);
        f.write(reinterpret_cast<const char*>(buf.data()),
                static_cast<std::streamsize>(rec));
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Photos
// ─────────────────────────────────────────────────────────────────────────────
static double parseTimestampNumber(double value) {
    const double a = std::fabs(value);
    if (a >= 1e15) return value / 1e9;
    if (a >= 1e11) return value / 1e3;
    return value;
}

double photoTimestampFromName(const std::string& stem) {
    if (stem.empty()) return -1.0;
    bool all_digits = true;
    for (char ch : stem)
        if (!std::isdigit(static_cast<unsigned char>(ch))) { all_digits = false; break; }
    if (all_digits) return parseTimestampNumber(std::stod(stem));

    std::vector<std::string> runs;
    std::string cur;
    for (char ch : stem) {
        if (std::isdigit(static_cast<unsigned char>(ch))) cur.push_back(ch);
        else if (!cur.empty()) { runs.push_back(cur); cur.clear(); }
    }
    if (!cur.empty()) runs.push_back(cur);
    if (runs.empty()) return -1.0;
    std::string best = runs.front();           // ties -> first (like max(key=len))
    for (const std::string& r : runs)
        if (r.size() > best.size()) best = r;
    return parseTimestampNumber(std::stod(best));
}

std::vector<Photo> listPhotos(const std::string& dir) {
    std::vector<Photo> ph;
    std::error_code ec;
    for (const fs::directory_entry& e : fs::directory_iterator(dir, ec)) {
        if (!e.is_regular_file()) continue;
        std::string fn = e.path().filename().string();
        const std::size_t dot = fn.rfind('.');
        if (dot == std::string::npos) continue;
        const std::string ext = lowerStr(fn.substr(dot + 1));
        if (ext != "jpg" && ext != "jpeg" && ext != "png" && ext != "bmp")
            continue;
        const double ts = photoTimestampFromName(fn.substr(0, dot));
        if (ts >= 0.0) ph.push_back(Photo{ts, e.path().string()});
    }
    std::sort(ph.begin(), ph.end(),
              [](const Photo& a, const Photo& b) { return a.t < b.t; });
    return ph;
}