// src/software/app/components/core/IR_Tracker_KCF.cpp
#include "components/includes/IR_Tracker_KCF.hpp"
#include <iostream>
#include <algorithm>

namespace flir {

static constexpr int MIN_W = 24;
static constexpr int MIN_H = 24;

cv::Rect2f IR_Tracker_KCF::clamp_roi(const cv::Rect2f& r, const cv::Size& sz) {
    float x = std::clamp(r.x, 0.0f, (float)(sz.width  - 1));
    float y = std::clamp(r.y, 0.0f, (float)(sz.height - 1));
    float w = std::clamp(r.width ,  1.0f, (float)sz.width  - x);
    float h = std::clamp(r.height,  1.0f, (float)sz.height - y);
    return cv::Rect2f(x, y, w, h);
}

IR_Tracker_KCF::IR_Tracker_KCF() {
    tracker_.release();
}

bool IR_Tracker_KCF::init(const cv::Mat& pf, const cv::Rect2f& box) {
    if (pf.empty()) {
        std::cerr << "[KCF] init: empty frame\n";
        return false;
    }

    if (pf.type() != CV_8U && pf.type() != CV_8UC1) {
        std::cerr << "[KCF] init: invalid input type (need CV_8U 1-channel), got type="
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

        // GRAY 모드 KCF 파라미터
        cv::legacy::TrackerKCF::Params params;
        params.desc_pca  = cv::TrackerKCF::GRAY;
        params.desc_npca = cv::TrackerKCF::GRAY;
        params.compress_feature = true;
        params.compressed_size  = 1;  // 1보다 2 정도가 안전

        tracker_ = cv::legacy::TrackerKCF::create(params);

        bool ok = tracker_->init(pf, roi_d);
        std::cout << "[KCF] init " << (ok ? "OK " : "FAIL ")
                  << "ROI=(" << (int)roi_d.x << "," << (int)roi_d.y << ","
                  << (int)roi_d.width << "," << (int)roi_d.height << ")"
                  << " img=" << pf.cols << "x" << pf.rows
                  << " ch=" << pf.channels() << " type=" << pf.type() << "\n";
        return ok;
    } catch (const cv::Exception& e) {
        std::cerr << "[KCF] exception in init(): " << e.what() << "\n";
        tracker_.release();
        return false;
    }
}

bool IR_Tracker_KCF::update(const cv::Mat& pf, cv::Rect2f& out_box, float& score) {
    if (!tracker_) {
        score = 0.0f;
        return false;
    }
    if (pf.empty()) {
        std::cerr << "[KCF] update: empty frame\n";
        score = 0.0f;
        return false;
    }

    if (pf.type() != CV_8U && pf.type() != CV_8UC1) {
        std::cerr << "[KCF] update: invalid input type (need CV_8U 1-channel), got type="
                  << pf.type() << "\n";
        score = 0.0f;
        return false;
    }

    cv::Rect2d tmp(out_box.x, out_box.y, out_box.width, out_box.height);

    bool ok = false;
    try {
        ok = tracker_->update(pf, tmp);
    } catch (const cv::Exception& e) {
        std::cerr << "[KCF] exception in update(): " << e.what()
                  << " (img=" << pf.cols << "x" << pf.rows
                  << " ch=" << pf.channels() << " type=" << pf.type() << ")\n";
        tracker_.release();
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
