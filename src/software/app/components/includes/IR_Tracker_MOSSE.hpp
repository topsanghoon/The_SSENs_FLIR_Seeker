// src/software/app/components/includes/IR_Tracker_MOSSE.hpp
#pragma once
#include <opencv2/core.hpp>
#include <opencv2/tracking/tracking_legacy.hpp>
#include "threads_includes/IR_TrackThread.hpp"

namespace flir {

// 실제 동작하는 MOSSE 트래커
class IR_Tracker_MOSSE : public ITrackerStrategy {
public:
    IR_Tracker_MOSSE();

    // ITrackerStrategy 인터페이스 구현
    bool init(const cv::Mat& pf, const cv::Rect2f& box) override;
    bool update(const cv::Mat& pf, cv::Rect2f& out_box, float& score) override;

private:
    cv::Ptr<cv::legacy::TrackerMOSSE> tracker_;
    cv::Rect2f clamp_roi(const cv::Rect2f& r, const cv::Size& sz);
};

} // namespace flir
