// components/core/EO_ArucoDetector_OpenCV.cpp
#include "components/includes/EO_ArucoDetector_OpenCV.hpp"
#include <opencv2/aruco.hpp>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <vector>

namespace flir {

namespace {
    inline int makeOddGE3(int v) {
        if (v < 3) v = 3;
        if ((v & 1) == 0) ++v;
        return v;
    }
}

EO_ArucoDetector_OpenCV::EO_ArucoDetector_OpenCV(int dict_id) {
    dict_ = cv::aruco::getPredefinedDictionary(dict_id);
}

std::vector<IArucoDetector::Detection>
EO_ArucoDetector_OpenCV::detect(const cv::Mat& pf_gray8) {
    std::vector<Detection> out;
    if (pf_gray8.empty()) return out;

    // --- 1) 프레임 크기 기반 파라미터 계산 ---
    const int w = pf_gray8.cols;
    const int h = pf_gray8.rows;
    const int minSide = std::min(w, h);

    // int wMin  = makeOddGE3(std::max(3, minSide / 64));
    // int wMax  = makeOddGE3(std::max(wMin,
    //                                 std::min(minSide - 1,
    //                                          std::max(9, minSide / 12))));
    // int wStep = makeOddGE3(std::max(3, (wMax - wMin) / 3));
    // int minBorder = std::max(1, std::min(6, minSide / 60));

    const double imgPerimeter  = double(w + h) * 2.0;
    const double byPixelFloor  = std::max(0.0, 10.0 / std::max(1.0, imgPerimeter)); // ≥10px
    const double absoluteFloor = 0.010;                                             // 1 %
    const double baseMinRate   = std::max(byPixelFloor, absoluteFloor);

    // --- 2) 실제 사용하는 파라미터 람다 ---
    auto make_params =
        [&](double k, double approx, double minRate) -> cv::Ptr<cv::aruco::DetectorParameters>
    {
        auto p = cv::aruco::DetectorParameters::create();

        // adaptive threshold – 네가 말한 wMin/wMax/wStep 사용
        p->adaptiveThreshWinSizeMin  = 7;
        p->adaptiveThreshWinSizeMax  = 11;
        p->adaptiveThreshWinSizeStep = 4;
        p->adaptiveThreshConstant    = 5;

        // 기하 파라미터
        p->minMarkerPerimeterRate      = 0.04f;  // 0.006~0.02 권장
        p->maxMarkerPerimeterRate      = 4.0f;
        p->polygonalApproxAccuracyRate = 0.06;   // 0.02~0.08
        p->minCornerDistanceRate       = 0.05f;
        p->minDistanceToBorder         = 3;
        p->minOtsuStdDev               = 5.0f;
        p->minMarkerDistanceRate       = 0.05f;
        p->markerBorderBits            = 1;

        // 코너 정밀화
        p->cornerRefinementMethod        = cv::aruco::CORNER_REFINE_NONE;
        p->cornerRefinementWinSize       = 5;
        p->cornerRefinementMaxIterations = 10;
        p->cornerRefinementMinAccuracy   = 0.05;

        p->detectInvertedMarker = false;
        return p;
    };

    // --- 3) 탐지 시도 ---
    std::vector<std::vector<cv::Point2f>> corners, rejected;
    std::vector<int> ids;

    auto try_once = [&](double k, double approx, double minRate) -> bool {
        auto P = make_params(k, approx, minRate);
        corners.clear();
        ids.clear();
        rejected.clear();
        try {
            cv::aruco::detectMarkers(pf_gray8, dict_, corners, ids, P, rejected);
        } catch (const cv::Exception&) {
            return false;
        }
        return !ids.empty();
    };

    bool ok = try_once(/*k*/7.0, /*approx*/0.03, /*minRate*/baseMinRate);
    // 필요하면 완화 시도 한 번 더 열어도 됨
    // if (!ok) ok = try_once(3.0, 0.05, std::max(baseMinRate, 0.008));

    if (!ok) return out;

    // --- 4) Detection 포맷 변환 ---
    out.reserve(ids.size());
    for (size_t i = 0; i < ids.size(); ++i) {
        const auto& c = corners[i];
        if (c.size() != 4) continue;

        cv::Rect r = cv::boundingRect(c);
        if (r.width <= 0 || r.height <= 0) continue;

        Detection d;
        d.id      = ids[i];
        d.corners = { c[0], c[1], c[2], c[3] };
        d.bbox    = cv::Rect2f(
            static_cast<float>(r.x),
            static_cast<float>(r.y),
            static_cast<float>(r.width),
            static_cast<float>(r.height)
        );
        out.push_back(std::move(d));
    }

    return out;
}

} // namespace flir
