// components/includes/IController.hpp
#pragma once
#include <atomic>
#include "components/includes/ControlDTO.hpp"
#include "components/includes/TargetFusion.hpp"

namespace flir {

class IController {
public:
    virtual ~IController() = default;
    virtual CtrlCmd solve(const TargetFusion& tf) = 0;  // ★ now_ns() 제거
};

class ControllerLR : public IController {
public:
    ControllerLR(int frame_w, float deadzone_px)
        : frame_w_(frame_w), deadzone_(deadzone_px) {}

    void test_set_aruco_id(int id) {
        last_aruco_id_.store(id, std::memory_order_relaxed);
        src_.store(ObsSource::ARUCO, std::memory_order_relaxed);
    }
    void test_force_tracking() { src_.store(ObsSource::TRACKING, std::memory_order_relaxed); }

    CtrlCmd solve(const TargetFusion& tf) override {
        ObsSource s = src_.load(std::memory_order_relaxed);
        if (s == ObsSource::ARUCO) {
            const int id = last_aruco_id_.load(std::memory_order_relaxed);
            CtrlCmd c{};
            if (id == 1)      c.mode = 1;   // RIGHT
            else if (id == 2) c.mode = 1;   // LEFT
            else if (id == 3) { src_.store(ObsSource::TRACKING, std::memory_order_relaxed); c.mode = 0; }
            else              c.mode = 1;
            c.p1 = 5; c.p2 = 0.f; c.p3 = 0.f;
            return c;
        }
        auto b  = tf.last_box();
        float cx = b.x + b.width * 0.5f;
        float center = static_cast<float>(frame_w_) * 0.5f;
        float err = cx - center;

        CtrlCmd c{};
        if (err < -deadzone_)       c.mode = 1; // LEFT
        else if (err >  deadzone_)  c.mode = 1; // RIGHT
        else                        c.mode = 0; // CENTER
        c.p1 = err; c.p2 = 0.f; c.p3 = 0.f;
        return c;
    }

private:
    int   frame_w_{160};
    float deadzone_{4.f};
    std::atomic<ObsSource> src_{ObsSource::TRACKING};
    std::atomic<int>       last_aruco_id_{0};
};

} // namespace flir
