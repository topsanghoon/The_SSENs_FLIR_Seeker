// src/software/app/threads_includes/common.hpp
#pragma once
#include <chrono>

namespace flir {

struct ScopedTimer {
    using Clock = std::chrono::steady_clock;
    Clock::time_point start;
    double& out_ms;

    explicit ScopedTimer(double& ref)
        : start(Clock::now()), out_ms(ref) {}

    ~ScopedTimer() {
        auto end = Clock::now();
        out_ms = std::chrono::duration<double, std::milli>(end - start).count();
    }
};

} // namespace flir
