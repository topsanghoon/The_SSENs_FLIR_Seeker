// components/core/EO_ArucoDetector_OpenCV.cpp
#include "components/includes/EO_ArucoDetector_OpenCV.hpp"
#include <opencv2/aruco.hpp>
#include <opencv2/imgproc.hpp>
#include <cmath>
#include <type_traits>
#include <algorithm>
#include <string>
#include <iostream>

namespace flir {

EO_ArucoDetector_OpenCV::EO_ArucoDetector_OpenCV(int dict_id) {
    dict_   = cv::aruco::getPredefinedDictionary(dict_id);
    params_ = cv::aruco::DetectorParameters::create();

    params_->adaptiveThreshWinSizeMin    = 3;
    params_->adaptiveThreshWinSizeMax    = 23;
    params_->adaptiveThreshWinSizeStep   = 10;
    params_->adaptiveThreshConstant      = 7;

    params_->minMarkerPerimeterRate      = 0.03f;
    params_->maxMarkerPerimeterRate      = 4.0f;
    params_->polygonalApproxAccuracyRate = 0.03;     // double일 수 있음
    params_->minCornerDistanceRate       = 0.05f;
    params_->minDistanceToBorder         = 3;
    params_->minOtsuStdDev               = 5.0f;

    params_->minMarkerDistanceRate       = 0.05f;
    params_->markerBorderBits            = 1;

    params_->cornerRefinementMethod      = cv::aruco::CORNER_REFINE_SUBPIX;
    params_->cornerRefinementWinSize     = 5;
    params_->cornerRefinementMaxIterations = 30;
    params_->cornerRefinementMinAccuracy   = 0.01;

    params_->detectInvertedMarker = true;
}

void EO_ArucoDetector_OpenCV::tuneParamsForSize(const cv::Size& sz) {
    const int minSide = std::min(sz.width, sz.height);
    if (minSide <= 0) return;

    int wMin  = makeOddGE3(std::max(3, minSide / 64));
    int wMax  = makeOddGE3(std::max(wMin, std::min(minSide - 1, std::max(9, minSide / 12))));
    int wStep = makeOddGE3(std::max(3, (wMax - wMin) / 3));
    params_->adaptiveThreshWinSizeMin  = wMin;
    params_->adaptiveThreshWinSizeMax  = wMax;
    params_->adaptiveThreshWinSizeStep = wStep;

    params_->minDistanceToBorder = std::max(1, std::min(6, minSide / 60));

    using RateT = decltype(params_->minMarkerPerimeterRate);
    const RateT imgPerimeter   = RateT(sz.width + sz.height) * RateT(2);
    const RateT minPerPix      = RateT(10);                // 8~12 px
    const RateT byPixelFloor   = std::max(RateT(0), minPerPix / std::max(RateT(1), imgPerimeter));
    const RateT absoluteFloor  = RateT(0.01);              // 절대 하한 비율
    const RateT targetMinRate  = std::max(byPixelFloor, absoluteFloor);

    // ★ 기존값과 max하지 말고, 프레임 크기 기반으로 ‘명시 대입’
    params_->minMarkerPerimeterRate = targetMinRate;

    using MaxRateT = decltype(params_->maxMarkerPerimeterRate);
    params_->maxMarkerPerimeterRate = std::max(params_->maxMarkerPerimeterRate, MaxRateT(4.0));
}


template<typename T>
static inline void fix_pos(T& v, T def) {
    if constexpr (std::is_floating_point_v<T>) {
        if (!std::isfinite(v) || v <= T(0)) v = def;
    } else {
        if (v <= T(0)) v = def;
    }
}

template<typename T>
static inline void fix_nonneg(T& v, T def) {
    if constexpr (std::is_floating_point_v<T>) {
        if (!std::isfinite(v) || v < T(0)) v = def;
    } else {
        if (v < T(0)) v = def;
    }
}

void EO_ArucoDetector_OpenCV::sanitizeParams() {
    fix_pos    (params_->minMarkerPerimeterRate,
                static_cast<decltype(params_->minMarkerPerimeterRate)>(0.03));
    fix_pos    (params_->maxMarkerPerimeterRate,
                static_cast<decltype(params_->maxMarkerPerimeterRate)>(4.0));
    fix_pos    (params_->polygonalApproxAccuracyRate,
                static_cast<decltype(params_->polygonalApproxAccuracyRate)>(0.03));
    fix_nonneg (params_->minCornerDistanceRate,
                static_cast<decltype(params_->minCornerDistanceRate)>(0.05));
    fix_nonneg (params_->minMarkerDistanceRate,
                static_cast<decltype(params_->minMarkerDistanceRate)>(0.05));
    if (params_->minDistanceToBorder < 0) params_->minDistanceToBorder = 3;

    using AccT = decltype(params_->polygonalApproxAccuracyRate);
    if (!std::isfinite(params_->polygonalApproxAccuracyRate) ||
        params_->polygonalApproxAccuracyRate <= AccT(0)) {
        params_->polygonalApproxAccuracyRate = AccT(0.02);
    } else if (params_->polygonalApproxAccuracyRate < AccT(0.01)) {
        params_->polygonalApproxAccuracyRate = AccT(0.01);
    }

    // min/max 관계식 보정 (양쪽 타입 맞춰서)
    using MinRateT = decltype(params_->minMarkerPerimeterRate);
    using MaxRateT = decltype(params_->maxMarkerPerimeterRate);
    if (params_->maxMarkerPerimeterRate <= params_->minMarkerPerimeterRate) {
        params_->maxMarkerPerimeterRate =
            std::max(static_cast<MaxRateT>(params_->minMarkerPerimeterRate * static_cast<MinRateT>(4.0)),
                     static_cast<MaxRateT>(2.0));
    }

    if (params_->adaptiveThreshWinSizeMin < 3) params_->adaptiveThreshWinSizeMin = 3;
    if (params_->adaptiveThreshWinSizeMax < params_->adaptiveThreshWinSizeMin)
        params_->adaptiveThreshWinSizeMax = params_->adaptiveThreshWinSizeMin + 6;
    if (params_->adaptiveThreshWinSizeStep < 1) params_->adaptiveThreshWinSizeStep = 3;

    if ((params_->adaptiveThreshWinSizeMin  & 1) == 0) ++params_->adaptiveThreshWinSizeMin;
    if ((params_->adaptiveThreshWinSizeMax  & 1) == 0) ++params_->adaptiveThreshWinSizeMax;
    if ((params_->adaptiveThreshWinSizeStep & 1) == 0) ++params_->adaptiveThreshWinSizeStep;
}

std::vector<IArucoDetector::Detection>
EO_ArucoDetector_OpenCV::detect(const cv::Mat& pf_gray8) {
    std::vector<Detection> out;
    // return out;
    if (pf_gray8.empty()) return out;

    // 1) 사전 고정: ctor에서 이미 DICT_4X4_50로 생성했다면 여기서는 건드리지 않는다.
    // dict_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50); // (중복안전)

    // 2) 프레임 크기 기반 튜닝값 계산 (하지만 멤버에 "기록"하지 않음)
    const int w = pf_gray8.cols, h = pf_gray8.rows;
    const int minSide = std::min(w, h);
    auto makeOddGE3 = [](int v){ if (v < 3) v = 3; if ((v & 1)==0) ++v; return v; };

    const int winMin  = makeOddGE3(std::max(3, minSide/64));
    const int winMax  = makeOddGE3(std::max(winMin, std::min(minSide-1, std::max(9, minSide/12))));
    const int winStep = makeOddGE3(std::max(3, (winMax - winMin)/3));
    const int minBorder = std::max(1, std::min(6, minSide/60));

    const double imgPerimeter = double(w + h) * 2.0;
    const double byPixelFloor = std::max(0.0, 10.0 / std::max(1.0, imgPerimeter)); // ≥10px
    const double absoluteFloor = 0.010; // 1.0%
    const double baseMinRate = std::max(byPixelFloor, absoluteFloor);

    auto make_params = [&](double k, double approx, double minRate)->cv::Ptr<cv::aruco::DetectorParameters>{
        auto p = cv::aruco::DetectorParameters::create();
        // adaptive
        p->adaptiveThreshWinSizeMin  = winMin;
        p->adaptiveThreshWinSizeMax  = winMax;
        p->adaptiveThreshWinSizeStep = winStep;
        p->adaptiveThreshConstant    = k;

        // 기하
        p->minMarkerPerimeterRate      = (float)minRate; // 0.006~0.02 사이 권장
        p->maxMarkerPerimeterRate      = 4.0f;
        p->polygonalApproxAccuracyRate = approx;         // 0.02~0.08
        p->minCornerDistanceRate       = 0.05f;
        p->minDistanceToBorder         = minBorder;
        p->minOtsuStdDev               = 5.0f;
        p->minMarkerDistanceRate       = 0.05f;
        p->markerBorderBits            = 1;

        // 코너 정밀화
        p->cornerRefinementMethod        = cv::aruco::CORNER_REFINE_SUBPIX;
        p->cornerRefinementWinSize       = 5;
        p->cornerRefinementMaxIterations = 30;
        p->cornerRefinementMinAccuracy   = 0.01;
        p->detectInvertedMarker          = true;
        return p;
    };

    // 3) 두 번만 시도: (a) 기본 (b) 살짝 완화 — 둘 다 "로컬 파라미터"만 수정
    std::vector<std::vector<cv::Point2f>> corners, rejected;
    std::vector<int> ids;

    auto try_once = [&](double k, double approx, double minRate)->bool {
        auto P = make_params(k, approx, minRate);
        corners.clear(); ids.clear(); rejected.clear();
        try {
            cv::aruco::detectMarkers(pf_gray8, dict_, corners, ids, P, rejected);
        } catch (const cv::Exception&) {
            return false;
        }
        return !ids.empty();
    };

    bool ok = false;
    ok = try_once(/*k*/7.0, /*approx*/0.03, /*minRate*/baseMinRate);
    if (!ok) ok = try_once(/*k*/3.0, /*approx*/0.05, /*minRate*/std::max(baseMinRate, 0.008));

    // (디버깅에 도움: 후보 생성은 되는지)
    if (!ok) {

        return out;
    }

    // 4) 가장 큰 마커 하나 선택
    int best_i = -1; double best_area = -1.0;
    for (size_t i = 0; i < ids.size(); ++i) {
        if (corners[i].size() != 4) continue;
        cv::Rect r = cv::boundingRect(corners[i]);
        if (r.width <= 0 || r.height <= 0) continue;
        double area = 1.0 * r.width * r.height;
        if (area > best_area) { best_area = area; best_i = (int)i; }
    }
    if (best_i < 0) return out;

    const auto& c = corners[best_i];
    cv::Rect r = cv::boundingRect(c);
    Detection d;
    d.id = ids[best_i];
    d.corners = { c[0], c[1], c[2], c[3] };
    d.bbox = cv::Rect2f((float)r.x, (float)r.y, (float)r.width, (float)r.height);
    out.push_back(d);
    return out;
}

} // namespace flir
