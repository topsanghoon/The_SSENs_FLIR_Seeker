// common_log.hpp
#pragma once
#include <cstdio>
#include <cstdarg>
#include <sstream>
#include <string>
#include <string_view>

namespace flir::log {

#if defined(FLIR_LOG_DISABLE)

// ----- 로그 완전 비활성 (컴파일 타임) -----
inline void logf(const char*, const char*, const char*, ...) {}

struct StreamGuardDisabled {
    std::ostringstream oss;                 // 체이닝 문법을 위해 남겨둠
    StreamGuardDisabled() = default;
    // ✅ 활성 버전과 동일한 시그니처를 받아들이는 더미 생성자 추가
    StreamGuardDisabled(const char*, const char*) {}
    ~StreamGuardDisabled() = default;
};

using StreamGuard = StreamGuardDisabled;

#else

// ----- 로그 활성 -----
inline void vlogf(const char* level, const char* tag, const char* fmt, va_list ap) {
    std::fputc('[', stdout);
    std::fputs(level, stdout);
    std::fputc(']', stdout);
    std::fputc('[', stdout);
    std::fputs(tag, stdout);
    std::fputs("] ", stdout);
    std::vfprintf(stdout, fmt, ap);
    std::fputc('\n', stdout);
    std::fflush(stdout);
}

inline void logf(const char* level, const char* tag, const char* fmt, ...) {
    va_list ap; va_start(ap, fmt);
    vlogf(level, tag, fmt, ap);
    va_end(ap);
}

struct StreamGuard {
    const char* level;
    const char* tag;
    std::ostringstream oss;
    StreamGuard(const char* lv, const char* tg) : level(lv), tag(tg) {}
    ~StreamGuard() {
        std::string s = oss.str();
        std::fputc('[', stdout);
        std::fputs(level, stdout);
        std::fputc(']', stdout);
        std::fputc('[', stdout);
        std::fputs(tag, stdout);
        std::fputs("] ", stdout);
        std::fwrite(s.data(), 1, s.size(), stdout);
        std::fputc('\n', stdout);
        std::fflush(stdout);
    }
};

#endif // FLIR_LOG_DISABLE

} // namespace flir::log

// =================== 매크로 ===================
// printf 스타일
#if defined(FLIR_LOG_DISABLE)
#  define LOGD(TAG, FMT, ...)   ((void)0)
#  define LOGI(TAG, FMT, ...)   ((void)0)
#  define LOGW(TAG, FMT, ...)   ((void)0)
#  define LOGE(TAG, FMT, ...)   ((void)0)
#else
#  define LOGD(TAG, FMT, ...)   ::flir::log::logf("D", TAG, FMT, ##__VA_ARGS__)
#  define LOGI(TAG, FMT, ...)   ::flir::log::logf("I", TAG, FMT, ##__VA_ARGS__)
#  define LOGW(TAG, FMT, ...)   ::flir::log::logf("W", TAG, FMT, ##__VA_ARGS__)
#  define LOGE(TAG, FMT, ...)   ::flir::log::logf("E", TAG, FMT, ##__VA_ARGS__)
#endif

// stream 스타일 (예: LOGDs("Net") << "x=" << x;)
#if defined(FLIR_LOG_DISABLE)
#  define LOGDs(TAG)            if (true) {} else ::flir::log::StreamGuard("D", TAG).oss
#  define LOGIs(TAG)            if (true) {} else ::flir::log::StreamGuard("I", TAG).oss
#  define LOGWs(TAG)            if (true) {} else ::flir::log::StreamGuard("W", TAG).oss
#  define LOGEs(TAG)            if (true) {} else ::flir::log::StreamGuard("E", TAG).oss
#else
#  define LOGDs(TAG)            ::flir::log::StreamGuard("D", TAG).oss
#  define LOGIs(TAG)            ::flir::log::StreamGuard("I", TAG).oss
#  define LOGWs(TAG)            ::flir::log::StreamGuard("W", TAG).oss
#  define LOGEs(TAG)            ::flir::log::StreamGuard("E", TAG).oss
#endif

/*
target_compile_definitions(rxtx_test PRIVATE
  $<$<CONFIG:Release>:FLIR_LOG_DISABLE>
)
*/