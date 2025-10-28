// components/core/EO_ArucoDetector_OpenCV.cpp
#include "components/includes/EO_ArucoDetector_OpenCV.hpp"
#include <opencv2/aruco.hpp>
#include <opencv2/imgproc.hpp>   // ★ 추가: boundingRect, rectangle 등

namespace flir {

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
    std::vector<std::vector<cv::Point2f>> corners;
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> rejected;

    cv::aruco::detectMarkers(pf_gray8, dict_, corners, ids, params_, rejected);

    std::vector<Detection> out;
    if (ids.empty()) return out;

    int best_i = -1;
    double best_area = -1.0;
    for (size_t i = 0; i < ids.size(); ++i) {
        const auto& c = corners[i];
        if (c.size() != 4) continue;
        cv::Rect r = cv::boundingRect(c);          // imgproc.hpp 필요
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
