// src/software/app/components/core/IR_Tracker_CSRT.cpp
#include "components/includes/IR_Tracker_CSRT.hpp"
#include <iostream>
#include <algorithm>

namespace flir {

static constexpr int MIN_W = 24;
static constexpr int MIN_H = 24;

cv::Rect2f IR_Tracker_CSRT::clamp_roi(const cv::Rect2f& r, const cv::Size& sz) {
    float x = std::clamp(r.x, 0.0f, (float)(sz.width  - 1));
    float y = std::clamp(r.y, 0.0f, (float)(sz.height - 1));
    float w = std::clamp(r.width ,  1.0f, (float)sz.width  - x);
    float h = std::clamp(r.height,  1.0f, (float)sz.height - y);
    return cv::Rect2f(x, y, w, h);
}

IR_Tracker_CSRT::IR_Tracker_CSRT() {
    // 실제 트래커 생성은 init()에서
    tracker_.release();
}

bool IR_Tracker_CSRT::init(const cv::Mat& pf, const cv::Rect2f& box) {
    if (pf.empty()) {
        std::cerr << "[CSRT] init: empty frame\n";
        return false;
    }

    // 🔥 전처리에서 이미 CV_8UC1로 맞춰졌다고 가정
    if (pf.type() != CV_8U && pf.type() != CV_8UC1) {
        std::cerr << "[CSRT] init: invalid input type (need CV_8U 1-channel), got type="
                  << pf.type() << "\n";
        return false;
    }

    cv::Rect2f roi_f = clamp_roi(box, pf.size());
    if (roi_f.width < MIN_W || roi_f.height < MIN_H) {
        roi_f.width  = std::max<float>(roi_f.width,  MIN_W);
        roi_f.height = std::max<float>(roi_f.height, MIN_H);
        roi_f = clamp_roi(roi_f, pf.size());
    }
    cv::Rect2d roi_d(roi_f.x, roi_f.y, roi_f.width, roi_f.height);

    try {
        tracker_.release();

        // 기본 파라미터로 legacy CSRT 생성
        tracker_ = cv::legacy::TrackerCSRT::create();

        bool ok = tracker_->init(pf, roi_d);  // legacy API: bool + Rect2d
        std::cout << "[CSRT] init " << (ok ? "OK " : "FAIL ")
                  << "ROI=(" << (int)roi_d.x << "," << (int)roi_d.y << ","
                  << (int)roi_d.width << "," << (int)roi_d.height << ")"
                  << " img=" << pf.cols << "x" << pf.rows
                  << " ch=" << pf.channels() << " type=" << pf.type() << "\n";
        return ok;
    } catch (const cv::Exception& e) {
        std::cerr << "[CSRT] exception in init(): " << e.what() << "\n";
        tracker_.release();
        return false;
    }
}

bool IR_Tracker_CSRT::update(const cv::Mat& pf, cv::Rect2f& out_box, float& score) {
    if (!tracker_) {
        score = 0.0f;
        return false;
    }
    if (pf.empty()) {
        std::cerr << "[CSRT] update: empty frame\n";
        score = 0.0f;
        return false;
    }

    if (pf.type() != CV_8U && pf.type() != CV_8UC1) {
        std::cerr << "[CSRT] update: invalid input type (need CV_8U 1-channel), got type="
                  << pf.type() << "\n";
        score = 0.0f;
        return false;
    }

    cv::Rect2d tmp(out_box.x, out_box.y, out_box.width, out_box.height);

    bool ok = false;
    try {
        ok = tracker_->update(pf, tmp);   // legacy API: bool + Rect2d&
    } catch (const cv::Exception& e) {
        std::cerr << "[CSRT] exception in update(): " << e.what()
                  << " (img=" << pf.cols << "x" << pf.rows
                  << " ch=" << pf.channels() << " type=" << pf.type() << ")\n";
        tracker_.release();  // 한 번 터지면 이 인스턴스는 버리고 다음 init 때 새로
        ok = false;
    }

    if (ok) {
        cv::Rect2f f((float)tmp.x, (float)tmp.y, (float)tmp.width, (float)tmp.height);
        f = clamp_roi(f, pf.size());
        f.width  = std::max<float>(f.width,  MIN_W);
        f.height = std::max<float>(f.height, MIN_H);
        out_box  = f;
    }

    score = ok ? 1.0f : 0.0f;
    return ok;
}

} // namespace flir
