// src/software/app/components/core/IR_Tracker_MOSSE.cpp
#include "components/includes/IR_Tracker_MOSSE.hpp"
#include <iostream>
#include <algorithm>

namespace flir {

static constexpr int MIN_W = 24;
static constexpr int MIN_H = 24;

cv::Rect2f IR_Tracker_MOSSE::clamp_roi(const cv::Rect2f& r, const cv::Size& sz) {
    float x = std::clamp(r.x, 0.0f, (float)(sz.width  - 1));
    float y = std::clamp(r.y, 0.0f, (float)(sz.height - 1));
    float w = std::clamp(r.width ,  1.0f, (float)sz.width  - x);
    float h = std::clamp(r.height,  1.0f, (float)sz.height - y);
    return cv::Rect2f(x, y, w, h);
}

IR_Tracker_MOSSE::IR_Tracker_MOSSE() {
    tracker_ = cv::legacy::TrackerMOSSE::create();
}

bool IR_Tracker_MOSSE::init(const cv::Mat& pf, const cv::Rect2f& box) {
    if (pf.empty()) {
        std::cerr << "[MOSSE] init: empty frame\n";
        return false;
    }
    if (pf.type() != CV_8U && pf.type() != CV_8UC1) {
        std::cerr << "[MOSSE] init: invalid input type (must be CV_8U 1-channel), type="
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
        tracker_ = cv::legacy::TrackerMOSSE::create();
        bool ok = tracker_->init(pf, roi_d);
        std::cout << "[MOSSE] init " << (ok ? "OK " : "FAIL ")
                  << "ROI=(" << (int)roi_d.x << "," << (int)roi_d.y << ","
                  << (int)roi_d.width << "," << (int)roi_d.height << "), "
                  << "img=" << pf.cols << "x" << pf.rows
                  << " ch=" << pf.channels() << " type=" << pf.type() << "\n";
        return ok;
    } catch (const cv::Exception& e) {
        std::cerr << "[MOSSE] exception in init(): " << e.what() << "\n";
        tracker_.release();
        return false;
    }
}

bool IR_Tracker_MOSSE::update(const cv::Mat& pf, cv::Rect2f& out_box, float& score) {
    if (!tracker_) return false;
    if (pf.empty()) {
        std::cerr << "[MOSSE] update: empty frame\n";
        score = 0.0f;
        return false;
    }
    if (pf.type() != CV_8U && pf.type() != CV_8UC1) {
        std::cerr << "[MOSSE] update: invalid input type (must be CV_8U 1-channel), type="
                  << pf.type() << "\n";
        score = 0.0f;
        return false;
    }

    cv::Rect2d tmp(out_box.x, out_box.y, out_box.width, out_box.height);

    bool ok = false;
    try {
        ok = tracker_->update(pf, tmp);
    } catch (const cv::Exception& e) {
        std::cerr << "[MOSSE] exception in update(): " << e.what() << "\n";
        tracker_.release();
        ok = false;
    }

    if (ok) {
        cv::Rect2f f((float)tmp.x, (float)tmp.y, (float)tmp.width, (float)tmp.height);
        f = clamp_roi(f, pf.size());
        f.width  = std::max<float>(f.width,  MIN_W);
        f.height = std::max<float>(f.height, MIN_H);
        out_box = f;
    }

    score = ok ? 1.0f : 0.0f;
    return ok;
}

} // namespace flir
