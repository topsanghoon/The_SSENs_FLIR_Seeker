// app/components/util/time_util.hpp
#pragma once

#include <cstdint>
#include <chrono>

#ifndef FLIR_PROFILE_ENABLED
#define FLIR_PROFILE_ENABLED 0
#endif

namespace flir {

// 시스템 시간(ms) - 필요하면 UI나 로그(별도)에서 사용
inline std::uint64_t now_ms_epoch() {
    using namespace std::chrono;
    return duration_cast<milliseconds>(
               system_clock::now().time_since_epoch())
        .count();
}

// steady 기준 ms - 상대 측정용
inline std::uint64_t now_ms_steady() {
    using namespace std::chrono;
    return duration_cast<milliseconds>(
               steady_clock::now().time_since_epoch())
        .count();
}

// ★ steady 기준 마이크로초(us) 절대 시각
//   - CSV_LOG_TL 의 t0_us~t3_us는 이 값 그대로 넣어주면 됨
inline std::uint64_t now_us_steady() {
    using namespace std::chrono;
    return std::chrono::duration_cast<std::chrono::microseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

inline std::uint64_t now_ns_steady() {
    using namespace std::chrono;
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

#if FLIR_PROFILE_ENABLED

// ms 단위 간단 프로파일러 (원하면 유지)
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

// us 단위 경과시간 측정용
struct ScopedTimerUs {
    using Clock = std::chrono::steady_clock;
    Clock::time_point t0;
    std::uint64_t& out_us;
    explicit ScopedTimerUs(std::uint64_t& ref) : t0(Clock::now()), out_us(ref) {}
    ~ScopedTimerUs() {
        using namespace std::chrono;
        out_us = duration_cast<microseconds>(Clock::now() - t0).count();
    }
};

#else

struct ScopedTimerMs {
    explicit ScopedTimerMs(double& ref) { (void)ref; }
    ~ScopedTimerMs() = default;
};

struct ScopedTimerUs {
    explicit ScopedTimerUs(std::uint64_t& ref) { (void)ref; }
    ~ScopedTimerUs() = default;
};

#endif

} // namespace flir
