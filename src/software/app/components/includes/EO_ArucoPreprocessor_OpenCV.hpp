#pragma once
#include <opencv2/imgproc.hpp>
#include "components/includes/EO_Frame.hpp"
#include "threads_includes/EO_ArUcoThread.hpp" // IArucoPreprocessor

namespace flir {

class EO_ArucoPreprocessor_OpenCV : public IArucoPreprocessor {
public:
    // BGR8 입력을 GRAY8로 변환 (+옵션 필터 가능)
    void run(const EOFrameHandle& in, cv::Mat& pf_gray8);
};

} // namespace flir