#pragma once
#include <cstdint>

namespace flir {

// 제어 명령 DTO
struct CtrlCmd {
    int   mode{0};      // 0: SAFE, 1: RUN ...
    float p1{0.f};
    float p2{0.f};
    float p3{0.f};

    static CtrlCmd safe_pose() { return CtrlCmd{0, 0.f, 0.f, 0.f}; }
    int to_int() const { return mode; }
};

// 자폭/안전정지 명령 DTO
struct SelfDestructCmd {
    uint32_t seq{0};
    int      level{1};   // 임의의 등급
};

} // namespace flir
