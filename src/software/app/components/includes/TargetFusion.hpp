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

    cv::Rect2f last_box()   const { return last_box_; }
    float      last_score() const { return last_score_; }
    uint64_t   last_ts()    const { return last_ts_;   }

    uint32_t   last_event_seq() const { return last_seq_.load(); }
    void       bump_last_event_seq()  { last_seq_.fetch_add(1);  }

    void set_quiesce(bool q) { quiesce_.store(q); }
    bool quiesce() const     { return quiesce_.load(); }

    // ArUco 정보(컨트롤러가 필요시 참고)
    ObsSource  last_source()   const { return last_src_.load(); }
    int        last_marker_id()const { return last_marker_id_.load(); }

private:
    cv::Rect2f             last_box_{0,0,0,0};
    float                  last_score_{0.f};
    uint64_t               last_ts_{0};

    std::atomic<uint32_t>  last_seq_{0};
    std::atomic<bool>      quiesce_{false};

    std::atomic<ObsSource> last_src_{ObsSource::NONE};
    std::atomic<int>       last_marker_id_{0};
};

} // namespace flir
