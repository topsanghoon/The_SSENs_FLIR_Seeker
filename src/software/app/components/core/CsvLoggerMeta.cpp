#include "components/includes/CsvLoggerMeta.hpp"
#include <cstdio>

namespace flir {

CsvLoggerMeta::CsvLoggerMeta(const std::string& path) {
    fp_ = std::fopen(path.c_str(), "w");
    if (fp_) std::fprintf(fp_, "ts_ns,kind,seq\n");
}
CsvLoggerMeta::~CsvLoggerMeta() {
    if (fp_) std::fclose(fp_);
}

void CsvLoggerMeta::log_send(const char* kind, uint32_t seq, uint64_t ts_ns) {
    std::lock_guard<std::mutex> lk(m_);
    if (fp_) { std::fprintf(fp_, "%llu,%s,%u\n",
            (unsigned long long)ts_ns, kind, seq); std::fflush(fp_); }
}

void CsvLoggerMeta::log_timer(uint64_t ts_ns) {
    std::lock_guard<std::mutex> lk(m_);
    if (fp_) { std::fprintf(fp_, "%llu,HB,0\n",
            (unsigned long long)ts_ns); std::fflush(fp_); }
}

void CsvLoggerMeta::log_udp_err(int err_no, const char* where) {
    std::lock_guard<std::mutex> lk(m_);
    if (fp_) { std::fprintf(fp_, "ERR,%s,%d\n", where, err_no); std::fflush(fp_); }
}

} // namespace flir