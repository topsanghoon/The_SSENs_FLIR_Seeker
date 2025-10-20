#include "components/includes/CsvLoggerAru.hpp"
#include <iomanip>

namespace flir {

CsvLoggerAru::CsvLoggerAru(const std::string& path) {
    file_.open(path, std::ios::out | std::ios::app);
    if (file_.is_open()) file_ << "frame_id,event,count,ms\n";
}

CsvLoggerAru::~CsvLoggerAru() {
    if (file_.is_open()) file_.close();
}

void CsvLoggerAru::marker_found(uint32_t frame_id, size_t count, double ms) {
    if (file_.is_open()) file_ << frame_id << ",FOUND," << count << "," << ms << "\n";
}

void CsvLoggerAru::marker_lost(uint32_t frame_id, double ms) {
    if (file_.is_open()) file_ << frame_id << ",LOST,0," << ms << "\n";
}

} // namespace flir
