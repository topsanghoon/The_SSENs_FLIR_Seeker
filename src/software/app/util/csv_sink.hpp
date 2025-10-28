// app/components/util/csv_sink.hpp
#pragma once
#include <string>
#include <string_view>
#include <mutex>
#include <fstream>
#include <atomic>

#include "util/telemetry.hpp"   // FLIR_CSV_ENABLED 스위치
#include "util/common_log.hpp"  // LOGE/LOGW/...

namespace flir {

class CsvSink {
public:
    // 전역 싱글턴
    static CsvSink& instance();

    // (선택) 파일명 강제 지정. 지정 안 하면 실행파일 디렉토리에 기본 파일명 사용.
    void set_filename(const std::string& path);

    // 최소 공통 포맷 라이터 (스레드 안전)
    // level: "I/D/W/E", th: "IR_Tx" 등 스레드 태그, seq: 프레임/명령 시퀀스 등
    // t0..t2..total_ms: 원하면 단계별 시간, 아니면 0.0 넣어도 됨
    // note: 임의 문자열(추가 필드)
    void write_simple(std::string_view level,
                      std::string_view th,
                      unsigned long seq,
                      double t0_ms,
                      double t1_ms,
                      double t2_ms,
                      double total_ms,
                      std::string_view note);

private:
    CsvSink() = default;
    ~CsvSink();

    void ensure_open_();
    static std::string default_path_in_exec_dir_();
    static bool file_exists_(const std::string& p);

private:
    std::mutex mtx_;
    std::ofstream ofs_;
    std::string file_path_;
    std::atomic<bool> header_written_{false};
};

} // namespace flir
