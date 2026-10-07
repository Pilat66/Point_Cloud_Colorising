// parallel.h — tiny std::thread helper shared by the parallel pipeline.
//
// parallelForRange(n, jobs, f) runs f(lo, hi) over [0, n) in njobs disjoint
// chunks on disjoint threads (f must be exception-free and thread-safe for
// its range). With jobs <= 1 it degrades to a single f(0, n) call on the
// calling thread. No external dependencies.
#pragma once

#include <cstddef>
#include <algorithm>
#include <thread>
#include <vector>

template <class F>
void parallelForRange(std::size_t n, int njobs, F&& f) {
    if (n == 0) return;
    if (njobs <= 1) { f(0, n); return; }
    const std::size_t chunk = (n + static_cast<std::size_t>(njobs) - 1) /
                              static_cast<std::size_t>(njobs);
    std::vector<std::thread> pool;
    for (std::size_t lo = 0; lo < n; lo += chunk) {
        const std::size_t hi = std::min(n, lo + chunk);
        pool.emplace_back([&f, lo, hi] { f(lo, hi); });
    }
    for (std::thread& t : pool) t.join();
}
