#pragma once
#include <cstdio>
#include <mutex>
#include <string>
#include "components/includes/ControlDTO.hpp"

namespace flir {

// 제어 경로 CSV 로거(간이)
class CsvLoggerCtrl {
public:
    explicit CsvLoggerCtrl(const std::string& path);
    ~CsvLoggerCtrl();

    void ctrl_out(const CtrlCmd& c);
    void sd_req(uint32_t seq, int level);
    void sd_done();
    void phase(const char* p);

private:
    std::mutex m_;
    std::FILE* fp_{nullptr};
};

} // namespace flir
