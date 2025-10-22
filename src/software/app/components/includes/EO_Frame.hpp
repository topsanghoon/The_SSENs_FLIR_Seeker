// components/includes/EO_Frame.hpp
#pragma once
#include <cstdint>
#include <opencv2/core.hpp>

namespace flir {

struct FrameBGR8 {
    uint8_t* data{};
    int      width{};
    int      height{};
    int      step{};
    inline cv::Mat asMat() const noexcept {
        return cv::Mat(height, width, CV_8UC3, data, step);
    }
};

struct EOFrameHandle {
    virtual ~EOFrameHandle() = default;   // ★ 가상 소멸자
    FrameBGR8* p{};
    uint64_t   ts{};
    uint32_t   seq{};
    
    // 가상 retain/release: 기본은 no-op (파생형에서 필요 시 override)
    virtual void retain() {}
    virtual void release() {}
    
    // 팩토리 메서드 - 특정 타입의 EOFrameHandle을 생성
    template<typename T>
    static std::shared_ptr<T> make() {
        return std::make_shared<T>();
    }
};

} // namespace flir
