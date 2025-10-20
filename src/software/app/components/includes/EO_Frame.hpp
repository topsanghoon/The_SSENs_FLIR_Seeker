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
    FrameBGR8* p{};
    uint64_t   ts{};
    uint32_t   seq{};
    void retain();
    void release();
};

} // namespace flir
