// src/software/app/components/includes/IR_Tracker_KCF.hpp
#pragma once
#include <opencv2/core.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/tracking/tracking_legacy.hpp>
#include "threads_includes/IR_TrackThread.hpp"

namespace flir {

// KCF 기반 IR 트래커 (입력: CV_8UC1, IR_Preprocessor 출력)
class IR_Tracker_KCF : public ITrackerStrategy {
public:
    IR_Tracker_KCF();

    bool init(const cv::Mat& pf, const cv::Rect2f& box) override;
    bool update(const cv::Mat& pf, cv::Rect2f& out_box, float& score) override;

private:
    cv::Ptr<cv::legacy::TrackerKCF> tracker_;

    cv::Rect2f clamp_roi(const cv::Rect2f& r, const cv::Size& sz);
};

} // namespace flir
