#pragma once
#include <opencv2/core.hpp>
#include "threads_includes/IR_TrackThread.hpp" // IReinitHintPolicy 인터페이스

namespace flir {

class ReinitHint_FromLast : public IReinitHintPolicy {
public:
    enum class Bias { Auto, Left, Right, Up, Down, Center };

    // 외부에서 상황 정보 주입
    void setBias(Bias b)                     noexcept { bias_ = b; }
    void setVelocity(const cv::Point2f& v)   noexcept { vel_  = v; }
    void setFrameSize(const cv::Size& s)     noexcept { frame_ = s; }

    // IReinitHintPolicy
    cv::Rect2f suggest(const cv::Rect2f& last_box) override;

    // 파라미터 (필요시 튜닝)
    void setStepPx(float px)                 noexcept { step_px_ = px; }
    void setDamping(float k)                 noexcept { damp_ = k; }   // 0..1

private:
    Bias        bias_   = Bias::Auto;
    cv::Point2f vel_{0.f, 0.f};   // px/frame (최근 추정 속도)
    cv::Size    frame_{0, 0};     // clamp 용

    float step_px_ = 6.f;         // 편향/기본 이동 스텝
    float damp_    = 0.5f;        // 속도 감쇠(불안정 줄이기)

    static inline float clampf(float v, float lo, float hi) {
        return std::max(lo, std::min(v, hi));
    }
};

} // namespace flir
