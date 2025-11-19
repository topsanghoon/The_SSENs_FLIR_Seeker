// app/components/util/csv_sink.hpp
#pragma once

#include <string>
#include <string_view>
#include <mutex>
#include <fstream>
#include <atomic>
#include <cstdint>

namespace flir {

// 단일 CSV 싱글턴: 공통 포맷
//   thread, seq, t0_us, t1_us, t2_us, t3_us, t_total_us, note
class CsvSink {
public:
    // 전역 싱글턴
    static CsvSink& instance();

    // (선택) 파일명 강제 지정. 지정 안 하면 실행파일 디렉토리에 기본 파일명 사용.
    void set_filename(const std::string& path);

    // 공통 로깅 함수
    //  - thread   : 스레드 태그 ("IR.Track", "EO.Aruco" 등)
    //  - seq      : 시퀀스 번호 (프레임 번호 등)
    //  - t0~t3_us : steady_clock 기준 절대 us 타임스탬프 (미사용 시 0)
    //  - total_us : 0 이면 내부에서 (마지막 non-zero tN - t0_us) 자동 계산
    //  - note     : 부가 정보 (THREAD_START, DETECT_OK,id=23 등)
    void write_timeline(std::string_view thread,
                        std::uint64_t    seq,
                        std::uint64_t    t0_us,
                        std::uint64_t    t1_us,
                        std::uint64_t    t2_us,
                        std::uint64_t    t3_us,
                        std::uint64_t    total_us,
                        std::string_view note);

private:
    CsvSink();
    ~CsvSink();

    void ensure_open_();
    static std::string default_path_in_exec_dir_();
    static bool file_exists_(const std::string& p);

private:
    std::mutex      mtx_;
    std::ofstream   ofs_;
    std::string     file_path_;
    std::atomic<bool> header_written_{false};
};

} // namespace flir
