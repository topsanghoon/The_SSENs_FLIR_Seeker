// src/software/app/components/core/IR_Preprocessor_LinNorm.cpp
#include "components/includes/IR_Preprocessor_LinNorm.hpp"
#include <opencv2/imgproc.hpp>
#include <algorithm>

namespace flir {

void IR_Preprocessor::run(const IRFrame16& in16, cv::Mat& out32f) {
    // RAW 버퍼를 cv::Mat으로 래핑 (복사 없음)
    cv::Mat src(in16.height, in16.width, CV_16UC1, in16.data, in16.step);

    // 고정 스케일로만 16U -> 32F 변환 (정규화 없음)
    // 14bit 센서면 scale_=1/16383.f, 16bit 원값 그대로 쓰려면 scale_=1.0f
    src.convertTo(out32f, CV_32F, (double)scale_, (double)offset_);
}


} // namespace flir
