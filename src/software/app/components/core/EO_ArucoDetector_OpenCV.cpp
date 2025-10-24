// components/core/EO_ArucoDetector_OpenCV.cpp
#include "components/includes/EO_ArucoDetector_OpenCV.hpp"

namespace flir {

EO_ArucoDetector_OpenCV::EO_ArucoDetector_OpenCV(int dict_id) {
    dict_   = cv::makePtr<cv::aruco::Dictionary>(cv::aruco::getPredefinedDictionary(dict_id));
    params_ = cv::makePtr<cv::aruco::DetectorParameters>();

    // 필요시 params_ 조정 (adaptiveThreshWinSizeMin/Max, cornerRefinementMethod 등)
}

std::vector<IArucoDetector::Detection>
EO_ArucoDetector_OpenCV::detect(const cv::Mat& pf_gray8) {
    std::vector<std::vector<cv::Point2f>> corners;
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> rejected;

    cv::aruco::detectMarkers(pf_gray8, dict_, corners, ids, params_, rejected);

    std::vector<Detection> out;
    out.reserve(ids.size());

    for (size_t i=0; i<ids.size(); ++i) {
        const auto& c = corners[i];
        if (c.size() != 4) continue;

        // bounding rect 계산
        //cv::Rect2f r = cv::boundingRect(c);
        cv::Rect2f r;

        Detection d;
        d.id = ids[i];
        d.corners = { c[0], c[1], c[2], c[3] };
        d.bbox = r;
        out.push_back(d);
    }
    return out;
}

} // namespace flir
