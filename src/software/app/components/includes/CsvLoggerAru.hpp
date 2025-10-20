#pragma once
#include <fstream>
#include <string>

namespace flir {

class CsvLoggerAru {
public:
    explicit CsvLoggerAru(const std::string& path = "/tmp/aruco_log.csv");
    ~CsvLoggerAru();

    void marker_found(uint32_t frame_id, size_t count, double ms);
    void marker_lost (uint32_t frame_id, double ms);

private:
    std::ofstream file_;
};

} // namespace flir
