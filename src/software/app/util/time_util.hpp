// util/time_util.hpp
#pragma once
#include <cstdint>
#include <chrono>
#include "util/telemetry.hpp"

namespace flir {

inline uint64_t now_ms_epoch() {
    using namespace std::chrono;
    return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}

inline uint64_t now_ms_steady() {
    using namespace std::chrono;
    return duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
}

#if FLIR_PROFILE_ENABLED
struct ScopedTimerMs {
    using Clock = std::chrono::steady_clock;
    Clock::time_point t0;
    double& out_ms;
    explicit ScopedTimerMs(double& ref) : t0(Clock::now()), out_ms(ref) {}
    ~ScopedTimerMs() {
        using namespace std::chrono;
        out_ms = duration<double,std::milli>(Clock::now() - t0).count();
    }
};
#else
struct ScopedTimerMs {
    explicit ScopedTimerMs(double& ref) { (void)ref; }
    ~ScopedTimerMs() = default;
};
#endif

} // namespace flir
