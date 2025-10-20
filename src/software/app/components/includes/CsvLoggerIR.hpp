// src/software/app/components/includes/CsvLoggerIR.hpp
#pragma once
#include <fstream>
#include <string>
#include <opencv2/core.hpp>

namespace flir {

// CsvLoggerIR.hpp (상속/override 제거)
class CsvLoggerIR {
public:
    explicit CsvLoggerIR(const std::string& path="/tmp/ir_track.csv");
    ~CsvLoggerIR();

    void click(uint32_t click_id, const cv::Rect2f& box);
    void init_ok(uint32_t frame_id, double ms);
    void init_fail(int fail_streak, double ms);
    void track_ok(uint32_t frame_id, float score, double ms);
    void track_lost(int fail_streak, double ms);
private:
    std::ofstream file_;
};


} // namespace flir
