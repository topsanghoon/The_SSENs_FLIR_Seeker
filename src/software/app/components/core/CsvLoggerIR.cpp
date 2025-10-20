// src/software/app/components/core/CsvLoggerIR.cpp
#include "components/includes/CsvLoggerIR.hpp"
#include <fstream>
#include <iomanip>

namespace flir {

CsvLoggerIR::CsvLoggerIR(const std::string& path) {
    file_.open(path, std::ios::out | std::ios::app);
    if (file_.is_open()) {
        file_ << "frame_id,event,score,fail_streak,ms\n";
    }
}

CsvLoggerIR::~CsvLoggerIR() {
    if (file_.is_open()) file_.close();
}

void CsvLoggerIR::click(uint32_t click_id, const cv::Rect2f& box) {
    if (file_.is_open()) file_ << click_id << ",CLICK," << box.x << "," << box.y << ",0\n";
}

void CsvLoggerIR::init_ok(uint32_t frame_id, double ms) {
    if (file_.is_open()) file_ << frame_id << ",INIT_OK,1,0," << ms << "\n";
}

void CsvLoggerIR::init_fail(int fail_streak, double ms) {
    if (file_.is_open()) file_ << "-1,INIT_FAIL,0," << fail_streak << "," << ms << "\n";
}

void CsvLoggerIR::track_ok(uint32_t frame_id, float score, double ms) {
    if (file_.is_open()) file_ << frame_id << ",TRACK_OK," << score << ",0," << ms << "\n";
}

void CsvLoggerIR::track_lost(int fail_streak, double ms) {
    if (file_.is_open()) file_ << "-1,TRACK_LOST,0," << fail_streak << "," << ms << "\n";
}

} // namespace flir
