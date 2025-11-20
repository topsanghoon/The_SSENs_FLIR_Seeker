// src/software/app/components/includes/IR_Preprocessor.hpp
#pragma once
#include <opencv2/core.hpp>
#include "components/includes/IR_Frame.hpp"
#include "threads_includes/IR_TrackThread.hpp" // IPreprocessor 인터페이스 상속

namespace flir {

// IR_Preprocessor: 16bit IR → 동적 정규화 → 8bit 그레이
class IR_Preprocessor : public IPreprocessor {
public:
    // 옛날 scale/offset은 현재 로직에서는 사용하지 않지만,
    // 인터페이스 호환을 위해 일단 남겨둔다.
    explicit IR_Preprocessor(float scale = 1.0f/16383.0f, float offset = 0.0f)
        : scale_(scale), offset_(offset) {}

    // outMat: CV_8UC1로 나옴
    void run(const IRFrame16& in16, cv::Mat& outMat) override;

private:
    float scale_;
    float offset_;
};

} // namespace flir