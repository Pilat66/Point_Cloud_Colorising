// ─────────────────────────────────────────────────────────────────────────────
// voxel_grid.hpp — radius query accelerator, a C++ substitute for scipy's
// cKDTree.query_ball_point used by colorise_offline.py.
//
// The cloud is bucketed into an axis-aligned uniform grid. Every point becomes
// one entry (uint64 key = packed voxel indices, uint32 source index); entries
// are sorted once by key. A radius query visits every voxel whose cell
// intersects the query sphere (cheap lower_bound/upper_bound over the sorted
// key array) and keeps points with squared distance <= r^2 — exactly the
// query_ball_point semantics.
//
// Memory: 12 bytes per point. Build is O(N log N).
// ─────────────────────────────────────────────────────────────────────────────
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

#include "colorise.h"
#include "parallel.h"

class VoxelGrid {
public:
    // Cell shape: the grid is grown until the per-axis voxel index fits into
    // 21 bits (2^21 voxels per axis, far beyond any real scene).
    static constexpr int SHIFT = 21;
    static constexpr int MASK  = (1 << SHIFT) - 1;

    void build(const Cloud& cloud, uint64_t target_cells = 1000000, int jobs = 1) {
        entries_.clear();
        const size_t n = cloud.n();
        if (n == 0) return;

        double min_x = cloud.x[0], max_x = cloud.x[0];
        double min_y = cloud.y[0], max_y = cloud.y[0];
        double min_z = cloud.z[0], max_z = cloud.z[0];
        for (size_t i = 1; i < n; ++i) {
            min_x = std::min(min_x, cloud.x[i]); max_x = std::max(max_x, cloud.x[i]);
            min_y = std::min(min_y, cloud.y[i]); max_y = std::max(max_y, cloud.y[i]);
            min_z = std::min(min_z, cloud.z[i]); max_z = std::max(max_z, cloud.z[i]);
        }
        min_x_ = min_x; min_y_ = min_y; min_z_ = min_z;

        const double vol = (max_x - min_x) * (max_y - min_y) * (max_z - min_z);
        cell_ = vol > 0.0 ? std::cbrt(vol / static_cast<double>(target_cells)) : 1.0;
        if (cell_ < 1.0) cell_ = 1.0;

        // Grow the cell until the index range fits 21 bits per axis.
        while (true) {
            const double rx = std::max(1.0, (max_x - min_x) / cell_);
            const double ry = std::max(1.0, (max_y - min_y) / cell_);
            const double rz = std::max(1.0, (max_z - min_z) / cell_);
            if (rx <= MASK && ry <= MASK && rz <= MASK) break;
            cell_ *= 2.0;
        }

        // Voxel keys are computed in parallel (pure per-point arithmetic);
        // the sort stays single-threaded and deterministic, so the entries
        // sequence — and therefore every radiusQuery — is bit-identical to
        // the single-threaded build.
        entries_.resize(n);
        parallelForRange(n, jobs, [&](std::size_t lo, std::size_t hi) {
            for (std::size_t i = lo; i < hi; ++i) {
                const long long ix = static_cast<long long>(
                    std::floor((cloud.x[i] - min_x_) / cell_));
                const long long iy = static_cast<long long>(
                    std::floor((cloud.y[i] - min_y_) / cell_));
                const long long iz = static_cast<long long>(
                    std::floor((cloud.z[i] - min_z_) / cell_));
                entries_[i] = Entry{packKey(ix, iy, iz), static_cast<uint32_t>(i)};
            }
        });
        std::sort(entries_.begin(), entries_.end(),
                  [](const Entry& a, const Entry& b) { return a.key < b.key; });
    }

    bool empty() const { return entries_.empty(); }

    // All cloud indices whose point lies within radius r of (cx,cy,cz).
    void radiusQuery(const Cloud& cloud, double cx, double cy, double cz,
                     double r, std::vector<int>& out) const {
        out.clear();
        if (entries_.empty()) return;

        const long long i0 = std::max<long long>(0, static_cast<long long>(
            std::floor((cx - r - min_x_) / cell_)));
        const long long i1 = std::max<long long>(0, static_cast<long long>(
            std::floor((cx + r - min_x_) / cell_)));
        const long long j0 = std::max<long long>(0, static_cast<long long>(
            std::floor((cy - r - min_y_) / cell_)));
        const long long j1 = std::max<long long>(0, static_cast<long long>(
            std::floor((cy + r - min_y_) / cell_)));
        const long long k0 = std::max<long long>(0, static_cast<long long>(
            std::floor((cz - r - min_z_) / cell_)));
        const long long k1 = std::max<long long>(0, static_cast<long long>(
            std::floor((cz + r - min_z_) / cell_)));

        const double rr = r * r;
        for (long long i = i0; i <= i1; ++i)
            for (long long j = j0; j <= j1; ++j)
                for (long long k = k0; k <= k1; ++k) {
                    const uint64_t key = packKey(i, j, k);
                    const uint64_t lo = lowerBound(key);
                    const uint64_t hi = upperBound(key);
                    for (uint64_t e = lo; e < hi; ++e) {
                        const uint32_t idx = entries_[e].idx;
                        const double dx = cloud.x[idx] - cx;
                        const double dy = cloud.y[idx] - cy;
                        const double dz = cloud.z[idx] - cz;
                        if (dx * dx + dy * dy + dz * dz <= rr) out.push_back(static_cast<int>(idx));
                    }
                }
    }

private:
    struct Entry {
        uint64_t key;
        uint32_t idx;
    };

    static uint64_t packKey(long long i, long long j, long long k) {
        return (static_cast<uint64_t>(i) << (2 * SHIFT)) |
               (static_cast<uint64_t>(j) << SHIFT) |
               static_cast<uint64_t>(k);
    }

    size_t lowerBound(uint64_t key) const {
        size_t lo = 0, hi = entries_.size();
        while (lo < hi) {
            const size_t mid = lo + (hi - lo) / 2;
            if (entries_[mid].key < key) lo = mid + 1; else hi = mid;
        }
        return lo;
    }

    size_t upperBound(uint64_t key) const {
        size_t lo = 0, hi = entries_.size();
        while (lo < hi) {
            const size_t mid = lo + (hi - lo) / 2;
            if (entries_[mid].key <= key) lo = mid + 1; else hi = mid;
        }
        return lo;
    }

    std::vector<Entry> entries_;
    double cell_ = 1.0;
    double min_x_ = 0.0, min_y_ = 0.0, min_z_ = 0.0;
};