#pragma once
#include <cstdint>

namespace flir {

struct IRFrame16 {
    uint16_t* data{};  // H x W x 1 (GRAY16)
    int       width{};
    int       height{};
    int       step{};  // 한 행 바이트 수 (2*width 또는 패딩 포함)
};

struct IRFrameHandle { //todo
    IRFrame16* p{};
    uint64_t   ts{};    // ns
    uint32_t   seq{};   // 증가 시퀀스
    void retain();      // 필요 시 참조 증가
    void release();     // 반납(버퍼 풀 등)
};

} // namespace flir