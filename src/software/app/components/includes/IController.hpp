#pragma once
#include <atomic>
#include <iostream>
#include "components/includes/ControlDTO.hpp"
#include "components/includes/TargetFusion.hpp"

namespace flir {

class IController {
public:
    virtual ~IController() = default;
    virtual CtrlCmd solve(const TargetFusion& tf) = 0;
};

class ControllerLR : public IController {
public:
    // ✨ EO/IR 폭을 둘 다 받도록 수정
    ControllerLR(int eo_w, int ir_w, float deadzone_px)
        : eo_w_(eo_w), ir_w_(ir_w), deadzone_(deadzone_px) {}

    // 테스트용 도우미는 그대로 둠
    void test_set_aruco_id(int id) {
        last_aruco_id_.store(id, std::memory_order_relaxed);
        src_.store(ObsSource::ARUCO, std::memory_order_relaxed);
    }
    void test_force_tracking() { src_.store(ObsSource::TRACKING, std::memory_order_relaxed); }

    // 필요 시 런타임에도 갱신할 수 있도록 세터 제공(옵션)
    void set_frame_widths(int eo_w, int ir_w) { eo_w_ = eo_w; ir_w_ = ir_w; }
    void set_deadzone_px(float dz) { deadzone_ = dz; }

    CtrlCmd solve(const TargetFusion& tf) override {
        // 현재 사용 중인 관측 소스
        ObsSource s = src_.load(std::memory_order_relaxed);

        // 🔸 소스에 맞는 프레임 폭 선택
        const int frame_w = (s == ObsSource::ARUCO) ? eo_w_ : ir_w_;

        // 관측 박스 중심과 영상 중심 비교
        const auto b  = tf.last_box();
        const float cx = b.x + b.width * 0.5f;
        const float center = 0.5f * static_cast<float>(frame_w);
        const float div_val = frame_w / 80;
        const float err = cx - center;
        std::cout << frame_w << "\n";

        CtrlCmd c{};
        if      (err < -deadzone_) c.mode = -1; // LEFT
        else if (err >  deadzone_) c.mode =  1; // RIGHT
        else                       c.mode =  0; // CENTER

        // 필요하면 스케일 조정(간단히 1/2 유지)
        c.p1 = err; 
        c.p2 = 0.f; 
        c.p3 = 0.f;
        return c;
    }

private:
    // 소스별 프레임 폭 보관
    int   eo_w_{320};
    int   ir_w_{80};
    float deadzone_{4.f};

    // 현재 사용 소스(ARUCO=EO, TRACKING=IR)
    std::atomic<ObsSource> src_{ObsSource::TRACKING};
    std::atomic<int>       last_aruco_id_{0};
};

} // namespace flir
