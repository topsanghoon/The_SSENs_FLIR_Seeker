#pragma once
#include <cstdio>
#include <mutex>
#include <string>

namespace flir {

class CsvLoggerMeta {
public:
    explicit CsvLoggerMeta(const std::string& path = "Meta_TxThread_log.csv");
    ~CsvLoggerMeta();

    void log_send(const char* kind, uint32_t seq, uint64_t ts_ns); // kind: "track","aruco","ctrl"
    void log_timer(uint64_t ts_ns);                                // heartbeat
    void log_udp_err(int err_no, const char* where);

private:
    std::mutex m_;
    FILE* fp_ = nullptr;
};

} // namespace flir
