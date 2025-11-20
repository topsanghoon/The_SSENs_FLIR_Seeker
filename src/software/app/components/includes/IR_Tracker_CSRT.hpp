// src/software/app/components/includes/IR_Tracker_CSRT.hpp
#pragma once
#include <opencv2/core.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/tracking/tracking_legacy.hpp>
#include "threads_includes/IR_TrackThread.hpp"

namespace flir {

// CSRT 기반 IR 트래커 (입력: CV_8UC1, 전처리 출력 그대로 사용)
class IR_Tracker_CSRT : public ITrackerStrategy {
public:
    IR_Tracker_CSRT();

    // ITrackerStrategy 인터페이스 구현
    // pf: CV_8UC1
    bool init(const cv::Mat& pf, const cv::Rect2f& box) override;
    bool update(const cv::Mat& pf, cv::Rect2f& out_box, float& score) override;

private:
    cv::Ptr<cv::legacy::TrackerCSRT> tracker_;

    cv::Rect2f clamp_roi(const cv::Rect2f& r, const cv::Size& sz);
};

} // namespace flir
