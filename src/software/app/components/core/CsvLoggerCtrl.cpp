// core
#include "components/includes/CsvLoggerCtrl.hpp"

namespace flir {

CsvLoggerCtrl::CsvLoggerCtrl(const std::string& path) {
    fp_ = std::fopen(path.c_str(), "w");
    if (fp_) std::fprintf(fp_, "type,seq,level,mode,p1,p2,p3,phase\n");
}
CsvLoggerCtrl::~CsvLoggerCtrl() { if (fp_) std::fclose(fp_); }

void CsvLoggerCtrl::ctrl_out(const CtrlCmd& c) {
    std::lock_guard<std::mutex> lk(m_);
    if (!fp_) return;
    std::fprintf(fp_, "ctrl_out,,,,%d,%.3f,%.3f,%.3f,\n", c.mode, c.p1, c.p2, c.p3);
    std::fflush(fp_);
}
void CsvLoggerCtrl::sd_req(uint32_t seq, int level) {
    std::lock_guard<std::mutex> lk(m_);
    if (!fp_) return;
    std::fprintf(fp_, "sd_req,%u,%d,,,,,\n", seq, level);
    std::fflush(fp_);
}
void CsvLoggerCtrl::sd_done() {
    std::lock_guard<std::mutex> lk(m_);
    if (!fp_) return;
    std::fprintf(fp_, "sd_done,,,,,,,\n");
    std::fflush(fp_);
}
void CsvLoggerCtrl::phase(const char* p) {
    std::lock_guard<std::mutex> lk(m_);
    if (!fp_) return;
    std::fprintf(fp_, "phase,,,,,,,,%s\n", p ? p : "");
    std::fflush(fp_);
}

} // namespace flir
