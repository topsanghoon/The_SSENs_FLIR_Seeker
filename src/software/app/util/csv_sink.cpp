// app/components/util/csv_sink.cpp
#include "util/csv_sink.hpp"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <chrono>
#include <filesystem>

#if defined(__linux__)
  #include <unistd.h>
  #include <limits.h>
#endif

namespace fs = std::filesystem;

namespace flir {

// -------- 싱글턴 --------
CsvSink& CsvSink::instance() {
    static CsvSink g;
    return g;
}

CsvSink::~CsvSink() {
    std::lock_guard<std::mutex> lk(mtx_);
    if (ofs_.is_open()) ofs_.flush(), ofs_.close();
}

void CsvSink::set_filename(const std::string& path) {
    std::lock_guard<std::mutex> lk(mtx_);
    if (ofs_.is_open()) { ofs_.flush(); ofs_.close(); }
    file_path_ = path;
    header_written_.store(false);
}

// -------- 내부 유틸 --------
bool CsvSink::file_exists_(const std::string& p) {
    std::error_code ec;
    return fs::exists(p, ec);
}

std::string CsvSink::default_path_in_exec_dir_() {
#if defined(__linux__)
    char buf[PATH_MAX] = {0};
    ssize_t n = ::readlink("/proc/self/exe", buf, sizeof(buf)-1);
    if (n > 0) {
        buf[n] = 0;
        fs::path exe = fs::path(buf);
        fs::path dir = exe.parent_path();
        // 항상 동일 파일명으로 append (요구사항: 하나의 CSV에 누적)
        return (dir / "flir_unified_log.csv").string();
    }
#endif
    // fallback: 현재 작업 디렉토리
    return (fs::current_path() / "flir_unified_log.csv").string();
}

void CsvSink::ensure_open_() {
#if FLIR_CSV_ENABLED
    if (ofs_.is_open()) return;

    if (file_path_.empty()) file_path_ = default_path_in_exec_dir_();

    // append 모드. 새 파일이면 헤더도 쓴다.
    const bool existed = file_exists_(file_path_);
    ofs_.open(file_path_, std::ios::out | std::ios::app);
    if (!ofs_) {
        LOGE("CSV", "open failed: %s", file_path_.c_str());
        return;
    }
    if (!existed) {
        ofs_ << "epoch_ms,level,thread,seq,t0_ms,t1_ms,t2_ms,total_ms,note\n";
        ofs_.flush();
        header_written_.store(true);
    } else {
        header_written_.store(true); // 이미 있었으면 헤더 있다고 가정
    }
#else
    // CSV 비활성일 때는 열지 않음
#endif
}

// -------- 퍼블릭 라이터 --------
void CsvSink::write_simple(std::string_view level,
                           std::string_view th,
                           unsigned long seq,
                           double t0_ms,
                           double t1_ms,
                           double t2_ms,
                           double total_ms,
                           std::string_view note) {
#if FLIR_CSV_ENABLED
    // 실행 오버헤드 아주 작음
    using namespace std::chrono;
    const uint64_t epoch_ms =
        duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();

    std::lock_guard<std::mutex> lk(mtx_);
    ensure_open_();
    if (!ofs_) return;

    // CSV-safe: 간단히 따옴표로 감싸는 정도만
    auto csv_escape = [](std::string_view s)->std::string {
        std::string out;
        out.reserve(s.size()+2);
        out.push_back('"');
        for (char c: s) {
            if (c=='"') out.push_back('"'); // CSV 의 이스케이프: "" 로 변환
            out.push_back(c);
        }
        out.push_back('"');
        return out;
    };

    ofs_ << epoch_ms << ','
         << csv_escape(level)  << ','
         << csv_escape(th)     << ','
         << seq                << ','
         << t0_ms              << ','
         << t1_ms              << ','
         << t2_ms              << ','
         << total_ms           << ','
         << csv_escape(note)   << '\n';
    ofs_.flush();
#else
    (void)level; (void)th; (void)seq; (void)t0_ms; (void)t1_ms; (void)t2_ms; (void)total_ms; (void)note;
#endif
}

} // namespace flir
