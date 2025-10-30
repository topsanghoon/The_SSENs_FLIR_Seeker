#pragma once
#include <cstdint>
#include <chrono>
#include <atomic>
#include <iostream>
#include "components/includes/ControlDTO.hpp"
#include "components/includes/TargetFusion.hpp" // ★ 여기에서 ObsSource를 가져옴 (재정의 금지)

namespace flir {

class IController {
public:
    virtual ~IController() = default;
    virtual CtrlCmd  solve(const TargetFusion& tf) = 0;
    virtual uint64_t now_ns() const = 0;
};

// ★ ObsSource 정의는 TargetFusion.hpp에만 존재해야 함
//   -> 여기서 다시 정의(duplicate)하지 말 것!

// 좌/우 제어 컨트롤러
class ControllerLR : public IController {
public:
    ControllerLR(int frame_w, float deadzone_px)
        : frame_w_(frame_w), deadzone_(deadzone_px) {}

    // 테스트용: ArUco 감지 상태 주입(IPC와 무관하게 로직만 검증)
    void test_set_aruco_id(int id) {
        last_aruco_id_.store(id, std::memory_order_relaxed);
        src_.store(ObsSource::ARUCO, std::memory_order_relaxed);
    }
    void test_force_tracking() {
        src_.store(ObsSource::TRACKING, std::memory_order_relaxed);
    }

    CtrlCmd solve(const TargetFusion& tf) override {
        // 소스 우선순위: 테스트 주입(ARUCO) > TF가 들고있는 소스(TRACKING)
        ObsSource s = src_.load(std::memory_order_relaxed);
        if (s == ObsSource::ARUCO) {
            const int id = last_aruco_id_.load(std::memory_order_relaxed);
            CtrlCmd c{};
            if (id == 1)      c.mode = 1;   // RIGHT
            else if (id == 2) c.mode = 1;   // LEFT
            else if (id == 3) {              // TRACKING 복귀
                src_.store(ObsSource::TRACKING, std::memory_order_relaxed);
                c.mode = 0;
            } else            c.mode = 1;
            c.p1 = 5; c.p2 = 0.f; c.p3 = 0.f;
            return c;
        }
        // TRACKING: 박스 중심으로 좌/우 판정
        auto b  = tf.last_box();
        float cx = b.x + b.width * 0.5f;
        float center = static_cast<float>(frame_w_) * 0.5f;
        float err = cx - center;

        CtrlCmd c{};
        if (err < -deadzone_)       c.mode = 1; // LEFT
        else if (err >  deadzone_)  c.mode = 1; // RIGHT
        else                        c.mode = 0;  // CENTER
        c.p1 = err; c.p2 = 0.f; c.p3 = 0.f;
        return c;
    }

    uint64_t now_ns() const override {
        using namespace std::chrono;
        return duration_cast<nanoseconds>(steady_clock::now().time_since_epoch()).count();
    }

private:
    int   frame_w_{160};
    float deadzone_{4.f};
    std::atomic<ObsSource> src_{ObsSource::TRACKING}; // ★ TargetFusion.hpp에서 온 타입
    std::atomic<int>       last_aruco_id_{0};
};

} // namespace flir
