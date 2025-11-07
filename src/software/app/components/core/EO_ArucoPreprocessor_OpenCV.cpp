// components/core/EO_ArucoPreprocessor_OpenCV.cpp
#include "components/includes/EO_ArucoPreprocessor_OpenCV.hpp"

namespace flir {

void EO_ArucoPreprocessor_OpenCV::run(const EOFrameHandle& in, cv::Mat& pf_gray8) {
    // BGR8 → GRAY8
    cv::Mat src = in.p->asMat();
    cv::cvtColor(src, pf_gray8, cv::COLOR_BGR2GRAY);
    cv::medianBlur(pf_gray8, pf_gray8, 3);

    auto clahe = cv::createCLAHE(3.0, cv::Size(8,8));
    clahe->apply(pf_gray8, pf_gray8);

    // (선택) 약간의 선명화/이퀄라이즈
    // cv::GaussianBlur(pf_gray8, pf_gray8, cv::Size(3,3), 0);
    // cv::equalizeHist(pf_gray8, pf_gray8);
}

} // namespace flir
