// components/core/EO_ArucoDetector_OpenCV.cpp
#include "components/includes/EO_ArucoDetector_OpenCV.hpp"
#include <opencv2/aruco.hpp>
#include <opencv2/imgproc.hpp>

namespace flir {

static inline void sanitizeArucoParams(cv::Ptr<cv::aruco::DetectorParameters>& p, const cv::Size& sz) {
    if (!p) p = cv::aruco::DetectorParameters::create();

    // 기본 안전값
    if (p->adaptiveThreshWinSizeMin < 3) p->adaptiveThreshWinSizeMin = 3;
    if (p->adaptiveThreshWinSizeMax < 3) p->adaptiveThreshWinSizeMax = 23;
    if (p->adaptiveThreshWinSizeStep <= 0) p->adaptiveThreshWinSizeStep = 10;

    // 홀수 보장
    if ((p->adaptiveThreshWinSizeMin % 2) == 0) ++p->adaptiveThreshWinSizeMin;
    if ((p->adaptiveThreshWinSizeMax % 2) == 0) ++p->adaptiveThreshWinSizeMax;

    // min<=max 보장
    if (p->adaptiveThreshWinSizeMin > p->adaptiveThreshWinSizeMax)
        std::swap(p->adaptiveThreshWinSizeMin, p->adaptiveThreshWinSizeMax);

    // 너무 큰 윈도우면 이미지 크기에 맞춰 클램프(선택)
    int maxWin = std::min({ sz.width, sz.height, 63 }); // 63은 경험적 상한
    if (p->adaptiveThreshWinSizeMax > maxWin) p->adaptiveThreshWinSizeMax = (maxWin | 1); // 홀수

    // 음수 금지
    if (p->minMarkerDistanceRate < 0) p->minMarkerDistanceRate = 0.f;
}

EO_ArucoDetector_OpenCV::EO_ArucoDetector_OpenCV(int dict_id) {
    dict_   = cv::aruco::getPredefinedDictionary(dict_id);
    params_ = cv::aruco::DetectorParameters::create();

    params_->adaptiveThreshWinSizeMin = 3;
    params_->adaptiveThreshWinSizeMax = 23;
    params_->adaptiveThreshWinSizeStep = 10;
    params_->minMarkerPerimeterRate = 0.03f;
    params_->maxMarkerPerimeterRate = 4.0f;
    params_->polygonalApproxAccuracyRate = 0.03f;
    params_->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
    params_->minCornerDistanceRate = 0.05f;
    params_->minDistanceToBorder = 2;
    params_->markerBorderBits = 1;
    params_->perspectiveRemovePixelPerCell = 8;
    params_->maxErroneousBitsInBorderRate = 0.5f;
    params_->minOtsuStdDev = 5.0f;
}

std::vector<IArucoDetector::Detection>
EO_ArucoDetector_OpenCV::detect(const cv::Mat& pf_gray8) {
    std::vector<Detection> out;
    if (pf_gray8.empty() || pf_gray8.type() != CV_8UC1) return out;

    // 매 호출 보정 (params_가 외부 요인으로 깨져 있어도 복구)
    sanitizeArucoParams(params_, pf_gray8.size());

    try {
        std::vector<std::vector<cv::Point2f>> corners, rejected;
        std::vector<int> ids;

        cv::aruco::detectMarkers(pf_gray8, dict_, corners, ids, params_, rejected);

        if (ids.empty()) return out;

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

    } catch (const cv::Exception& e) {
        // 예외는 상위로 올리지 말고 빈 결과로 리턴 (스레드 생존)
        // 상위에서 LOGE/CSV로 기록
        return {};
    }
}

} // namespace flir
