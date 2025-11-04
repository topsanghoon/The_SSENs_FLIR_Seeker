// TargetFusion.hpp
#pragma once
#include <atomic>
#include <cstdint>
#include <opencv2/core.hpp>

namespace flir {

enum class ObsSource { NONE, TRACKING, ARUCO };

class TargetFusion {
public:
    void update_with_track(const cv::Rect2f& box, float score, uint64_t ts_ns, uint32_t /*frame_seq*/);
    void update_with_marker(int id, const cv::Rect2f& box, uint64_t ts_ns);
    void drain_for_ms(int ms);

    // 조회자
    cv::Rect2f last_box()   const { return last_box_; }
    float      last_score() const { return last_score_; }
    uint64_t   last_ts()    const { return last_ts_; }
    ObsSource  last_src()   const { return last_src_.load(); }
    int        last_marker_id() const { return last_marker_id_.load(); }

    void set_quiesce(bool q) { quiesce_.store(q); }
    bool quiesce() const { return quiesce_.load(); }

    // ★ ControlThread가 기대하는 인터페이스 추가
    uint32_t last_event_seq() const { return last_evt_seq_.load(std::memory_order_relaxed); }
    void bump_last_event_seq() { (void)last_evt_seq_.fetch_add(1, std::memory_order_relaxed); }

private:
    cv::Rect2f             last_box_{0,0,0,0};
    float                  last_score_{0.f};
    uint64_t               last_ts_{0};

    std::atomic<uint32_t>  last_seq_{0};      // (기존 유지: 내부 필요 시 사용)
    std::atomic<bool>      quiesce_{false};

    std::atomic<ObsSource> last_src_{ObsSource::NONE};
    std::atomic<int>       last_marker_id_{0};

    // ★ 신규: 이벤트 소비 카운터 (EventBus inbox 처리 진척도)
    std::atomic<uint32_t>  last_evt_seq_{0};
};

} // namespace flir
