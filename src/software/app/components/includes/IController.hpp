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

    virtual void on_aruco_event(int /*id*/) {}
    virtual void on_track_event() {}
};

class ControllerLR : public IController {
public:
    ControllerLR(int eo_w, int ir_w, float deadzone_px)
        : eo_w_(eo_w), ir_w_(ir_w), deadzone_(deadzone_px) {}

    void on_aruco_event(int id) override {
        last_aruco_id_.store(id, std::memory_order_relaxed);
        src_.store(ObsSource::ARUCO, std::memory_order_relaxed);
    }
    void on_track_event() override {
        src_.store(ObsSource::TRACKING, std::memory_order_relaxed);
    }

    CtrlCmd solve(const TargetFusion& tf) override {
        ObsSource s = src_.load(std::memory_order_relaxed);
        const int frame_w = (s == ObsSource::ARUCO) ? eo_w_ : ir_w_;

        const auto b  = tf.last_box();
        const float cx = b.x + b.width * 0.5f;
        const float center = 0.5f * static_cast<float>(frame_w);
        const float div_val = frame_w / 80;
        const float err = cx - center;

        CtrlCmd c{};
        if      (err < -deadzone_) c.mode = -1;
        else if (err >  deadzone_) c.mode =  1;
        else                       c.mode =  0;
        c.p1 = err / div_val; c.p2 = 0.f; c.p3 = 0.f;
        return c;
    }

private:
    int   eo_w_{320};
    int   ir_w_{80};
    float deadzone_{4.f};
    std::atomic<ObsSource> src_{ObsSource::TRACKING};
    std::atomic<int>       last_aruco_id_{0};
};

} // namespace flir
