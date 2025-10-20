// src/software/app/components/includes/IR_Preprocessor_LinNorm.hpp
#pragma once
#include <opencv2/core.hpp>
#include "IR_Frame.hpp"
#include "threads_includes/common.hpp"
#include "threads_includes/IR_TrackThread.hpp" // 인터페이스 상속용

namespace flir {

// IR_Preprocessor_LinNorm.hpp
class IR_Preprocessor_LinNorm : public IPreprocessor {
public:
    // 14bit 센서면 기본값을 1/16383.f, 16bit면 1/65535.f 또는 1.0f로 사용
    explicit IR_Preprocessor_LinNorm(float scale = 1.0f/16383.0f, float offset = 0.0f)
        : scale_(scale), offset_(offset) {}

    void run(const IRFrame16& in16, cv::Mat& out32f) override;

private:
    float scale_;
    float offset_;
};


} // namespace flir
