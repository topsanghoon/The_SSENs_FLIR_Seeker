// components/core/EO_ArucoDetector_OpenCV.cpp
#include "components/includes/EO_ArucoDetector_OpenCV.hpp"
#include <opencv2/aruco.hpp>
#include <opencv2/imgproc.hpp>   // ★ 추가: boundingRect, rectangle 등
#include <iostream>

#include <limits>

namespace flir {

// --- helper ---
static inline bool finite_pos(double v)    { return std::isfinite(v) && v >  0; }
static inline bool finite_nonneg(double v) { return std::isfinite(v) && v >= 0; }
static inline int  make_odd_at_least_3(int v) {
    if (v < 3) v = 3;
    if ((v & 1) == 0) v += 1;
    return v;
}

// ★ OpenCV 4.5.2 DetectorParameters 안전 범위로 모두 클램프
static inline void sanitize_params_for_image(cv::aruco::DetectorParameters& p, int w, int h) {
    const int minSide = std::max(3, std::min(w, h));

    // adaptive threshold
    p.adaptiveThreshWinSizeMin  = std::min(make_odd_at_least_3(p.adaptiveThreshWinSizeMin),  make_odd_at_least_3(minSide));
    p.adaptiveThreshWinSizeMax  = std::min(make_odd_at_least_3(p.adaptiveThreshWinSizeMax),  make_odd_at_least_3(minSide));
    p.adaptiveThreshWinSizeStep = std::max(1, p.adaptiveThreshWinSizeStep);
    if (p.adaptiveThreshWinSizeMin > p.adaptiveThreshWinSizeMax)
        std::swap(p.adaptiveThreshWinSizeMin, p.adaptiveThreshWinSizeMax);

    // perimeter / accuracy
    if (!finite_pos(p.minMarkerPerimeterRate))      p.minMarkerPerimeterRate      = 0.03; // >0
    if (!finite_pos(p.maxMarkerPerimeterRate))      p.maxMarkerPerimeterRate      = 4.0;  // >0
    if (p.maxMarkerPerimeterRate < p.minMarkerPerimeterRate)
        std::swap(p.minMarkerPerimeterRate, p.maxMarkerPerimeterRate);

    if (!finite_pos(p.polygonalApproxAccuracyRate)) p.polygonalApproxAccuracyRate = 0.03; // >0

    // distances
    if (!finite_nonneg(p.minCornerDistanceRate))    p.minCornerDistanceRate       = 0.05; // >=0
    if (!finite_nonneg(p.minDistanceToBorder))      p.minDistanceToBorder         = 3;    // >=0 (px)
    p.minDistanceToBorder = std::min(p.minDistanceToBorder, minSide/2);

    // ★ 문제의 필드: markers 간 최소 거리 비율
    if (!finite_nonneg(p.minMarkerDistanceRate))    p.minMarkerDistanceRate       = 0.05; // >=0 (0도 허용이지만 기본 0.05 권장)

    // misc
    if (p.markerBorderBits < 1)                     p.markerBorderBits            = 1;
    if (p.perspectiveRemovePixelPerCell < 1)        p.perspectiveRemovePixelPerCell = 4;
    if (!std::isfinite(p.perspectiveRemoveIgnoredMarginPerCell) || p.perspectiveRemoveIgnoredMarginPerCell < 0)
        p.perspectiveRemoveIgnoredMarginPerCell = 0.13;

    if (!finite_nonneg(p.maxErroneousBitsInBorderRate)) p.maxErroneousBitsInBorderRate = 0.35;
    if (!finite_nonneg(p.minOtsuStdDev))                p.minOtsuStdDev               = 5.0;
    if (!finite_pos(p.errorCorrectionRate))             p.errorCorrectionRate         = 0.6;

    // corner refinement
    if (p.cornerRefinementMethod < 0 || p.cornerRefinementMethod > cv::aruco::CORNER_REFINE_CONTOUR)
        p.cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
    if (p.cornerRefinementWinSize <= 0)                p.cornerRefinementWinSize     = 5;
    if (p.cornerRefinementMaxIterations <= 0)          p.cornerRefinementMaxIterations = 30;
    if (!finite_nonneg(p.cornerRefinementMinAccuracy)) p.cornerRefinementMinAccuracy = 0.01;
}


EO_ArucoDetector_OpenCV::EO_ArucoDetector_OpenCV(int dict_id) {
    dict_   = cv::aruco::getPredefinedDictionary(dict_id);
    params_ = cv::aruco::DetectorParameters::create();

    // 필요하면 파라미터 튜닝 예시:
    // params_->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
    // params_->adaptiveThreshWinSizeMin = 3;
    // params_->adaptiveThreshWinSizeMax = 23;
    // params_->adaptiveThreshWinSizeStep = 10;
}


std::vector<IArucoDetector::Detection>
EO_ArucoDetector_OpenCV::detect(const cv::Mat& pf_gray8) {
    std::vector<Detection> out;

    // (1) 입력 방어
    if (pf_gray8.empty() || pf_gray8.type() != CV_8UC1 || pf_gray8.cols < 3 || pf_gray8.rows < 3) return out;

    // (2) 파라미터: 매 호출마다 로컬 복사해서 정규화 (외부에서 0/NaN 덮여도 안전)
    cv::Ptr<cv::aruco::DetectorParameters> local = cv::aruco::DetectorParameters::create();
    *local = *params_; // 사용자가 세팅한 값을 복사
    sanitize_params_for_image(*local, pf_gray8.cols, pf_gray8.rows);

    // (3) 검출 with try/catch
    std::vector<std::vector<cv::Point2f>> corners, rejected;
    std::vector<int> ids;
    try {
        cv::aruco::detectMarkers(pf_gray8, dict_, corners, ids, local, rejected);
    } catch (const cv::Exception& e) {
        std::cerr << "[EO.Aruco] detectMarkers exception: " << e.what() << "\n";
        return out;
    } catch (const std::exception& e) {
        std::cerr << "[EO.Aruco] std exception: " << e.what() << "\n";
        return out;
    } catch (...) {
        std::cerr << "[EO.Aruco] unknown exception in detectMarkers\n";
        return out;
    }

    if (ids.empty()) return out;

    // (4) 결과 검증/클램프 (NaN/무한/음수 방지)
    int best_i = -1; double best_area = -1.0;
    for (size_t i = 0; i < ids.size(); ++i) {
        const auto& poly = corners[i];
        if (poly.size() != 4) continue;

        bool ok_pts = true;
        for (const auto& pt : poly) {
            if (!std::isfinite(pt.x) || !std::isfinite(pt.y)) { ok_pts = false; break; }
            if (pt.x < -1e6 || pt.x > 1e6 || pt.y < -1e6 || pt.y > 1e6) { ok_pts = false; break; }
        }
        if (!ok_pts) continue;

        cv::Rect r = cv::boundingRect(poly);
        if (r.width <= 0 || r.height <= 0) continue;
        r &= cv::Rect(0,0, pf_gray8.cols, pf_gray8.rows); // 프레임 내부로

        double area = 1.0 * r.width * r.height;
        if (area > best_area) { best_area = area; best_i = (int)i; }
    }
    if (best_i < 0) return out;

    const auto& c = corners[best_i];
    cv::Rect r = cv::boundingRect(c);
    r &= cv::Rect(0,0, pf_gray8.cols, pf_gray8.rows);

    Detection d;
    d.id = ids[best_i];
    d.corners = { c[0], c[1], c[2], c[3] };
    d.bbox = cv::Rect2f((float)r.x, (float)r.y, (float)r.width, (float)r.height);
    out.push_back(d);
    return out;
}



} // namespace flir
