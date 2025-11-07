// components/includes/EO_ArucoDetector_OpenCV.hpp
#pragma once
#include <opencv2/aruco.hpp>
#include "threads_includes/EO_ArUcoThread.hpp"

namespace flir {


class EO_ArucoDetector_OpenCV : public IArucoDetector {
public:
    explicit EO_ArucoDetector_OpenCV(int dict_id = cv::aruco::DICT_4X4_50);
    std::vector<Detection> detect(const cv::Mat& pf_gray8) override;

private:
    cv::Ptr<cv::aruco::Dictionary>        dict_;
    cv::Ptr<cv::aruco::DetectorParameters> params_;

    static inline int makeOddGE3(int v) {
        if (v < 3) v = 3;
        if ((v & 1) == 0) ++v;
        return v;
    }

    void tuneParamsForSize(const cv::Size& sz);
    void sanitizeParams();   // ★ 추가: 0/음수/NaN 방지
};

} // namespace flir
