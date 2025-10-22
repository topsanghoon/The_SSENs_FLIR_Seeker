// TargetFusion.cpp
#include <opencv2/core.hpp>
#include "components/includes/TargetFusion.hpp"
#include <thread>
#include <chrono>

namespace flir {

void TargetFusion::update_with_track(const cv::Rect2f& box, float score, uint64_t ts_ns, uint32_t /*frame_seq*/) {
    if (quiesce_.load()) return;
    last_box_ = box; last_score_ = score; last_ts_ = ts_ns;
    last_src_.store(ObsSource::TRACKING);
}

void TargetFusion::update_with_marker(int id, const cv::Rect2f& box, uint64_t ts_ns) {
    if (quiesce_.load()) return;
    last_box_ = box; last_ts_ = ts_ns;
    last_marker_id_.store(id);
    last_src_.store(ObsSource::ARUCO);
}

void TargetFusion::drain_for_ms(int ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

} // namespace flir
