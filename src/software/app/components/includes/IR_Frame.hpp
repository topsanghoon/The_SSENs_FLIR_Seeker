#pragma once
#include <cstdint>

namespace flir {

struct IRFrame16 {
    uint16_t* data{};  // H x W x 1 (GRAY16)
    int       width{};
    int       height{};
    int       step{};  // 한 행 바이트 수 (2*width 또는 패딩 포함)
};

struct IRFrameHandle {
    virtual ~IRFrameHandle() = default;   // ★ 가상 소멸자
    IRFrame16* p{};                       // 포인터만 보관(소유 X)
    uint64_t   ts{};                      // ns
    uint32_t   seq{};                     // seq

    // 가상 retain/release: 기본은 no-op (파생형에서 필요 시 override)
    virtual void retain() {}
    virtual void release() {}
};

} // namespace flir