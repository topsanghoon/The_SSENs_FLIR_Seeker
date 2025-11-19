// app/components/util/csv_sink.cpp
#include "util/csv_sink.hpp"

#include <chrono>
#include <filesystem>
#include <system_error>

#if defined(__linux__)
  #include <unistd.h>
  #include <limits.h>
#endif

#include "util/common_log.hpp"  // LOGE

namespace fs = std::filesystem;

namespace flir {

// -------- 싱글턴 --------
CsvSink& CsvSink::instance() {
    static CsvSink g;
    return g;
}

CsvSink::CsvSink() = default;

CsvSink::~CsvSink() {
    std::lock_guard<std::mutex> lk(mtx_);
    if (ofs_.is_open()) {
        ofs_.flush();
        ofs_.close();
    }
}

void CsvSink::set_filename(const std::string& path) {
    std::lock_guard<std::mutex> lk(mtx_);
    if (ofs_.is_open()) {
        ofs_.flush();
        ofs_.close();
    }
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
        return (dir / "flir_unified_log.csv").string();
    }
#endif
    // fallback: 현재 작업 디렉토리
    return (fs::current_path() / "flir_unified_log.csv").string();
}

void CsvSink::ensure_open_() {
    if (ofs_.is_open()) return;

    if (file_path_.empty()) file_path_ = default_path_in_exec_dir_();

    const bool existed = file_exists_(file_path_);
    ofs_.open(file_path_, std::ios::out | std::ios::app);
    if (!ofs_) {
        LOGE("CSV", "open failed: %s", file_path_.c_str());
        return;
    }

    if (!existed) {
        // 공통 헤더
        ofs_ << "thread,seq,"
             << "t0_us,t1_us,t2_us,t3_us,"
             << "t_total_us,"
             << "note\n";
        ofs_.flush();
        header_written_.store(true);
    } else {
        header_written_.store(true);
    }
}

// -------- 퍼블릭 라이터 --------
void CsvSink::write_timeline(std::string_view thread,
                             std::uint64_t    seq,
                             std::uint64_t    t0_us,
                             std::uint64_t    t1_us,
                             std::uint64_t    t2_us,
                             std::uint64_t    t3_us,
                             std::uint64_t    total_us,
                             std::string_view note) {
    std::lock_guard<std::mutex> lk(mtx_);
    ensure_open_();
    if (!ofs_) return;

    auto csv_escape = [](std::string_view s)->std::string {
        std::string out;
        out.reserve(s.size()+2);
        out.push_back('"');
        for (char c: s) {
            if (c=='"') out.push_back('"'); // "" 로 이스케이프
            out.push_back(c);
        }
        out.push_back('"');
        return out;
    };

    // total_us == 0 이면 자동 계산 (마지막 non-zero tN - t0)
    std::uint64_t total = total_us;
    if (total == 0 && t0_us != 0) {
        std::uint64_t last = t0_us;
        if (t1_us != 0) last = t1_us;
        if (t2_us != 0) last = t2_us;
        if (t3_us != 0) last = t3_us;

        if (last >= t0_us) total = last - t0_us;
        else total = 0;
    }

    ofs_ << csv_escape(thread) << ','
         << seq                << ','
         << t0_us              << ','
         << t1_us              << ','
         << t2_us              << ','
         << t3_us              << ','
         << total              << ','
         << csv_escape(note)   << '\n';
    ofs_.flush();
}

} // namespace flir
