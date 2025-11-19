// app/components/util/telemetry.hpp
#pragma once

// CSV 로깅 on/off 스위치
#ifndef FLIR_CSV_ENABLED
#define FLIR_CSV_ENABLED 1
#endif

#include "util/common_log.hpp"
#include "util/csv_sink.hpp"

namespace flir {
    // telemetry 관련 enum/구조체가 필요하면 여기 추가
}

// ─────────────────────────────────────────────
// 공통 CSV 타임라인 매크로
//  - 항상 다음 포맷으로 기록:
//      thread, seq, t0_us, t1_us, t2_us, t3_us, t_total_us, note
//  - T_TOTAL_US == 0 이면 내부에서 (마지막 non-zero tN - t0_us) 자동 계산
//  - THREAD_START/STOP 등도 note에 문자열로 넣고,
//    나머지 t0~t3, seq 는 0 또는 더미로 채워서 동일 포맷 사용.
// ─────────────────────────────────────────────
#if FLIR_CSV_ENABLED

  #define CSV_LOG_TL(THREAD, SEQ, T0_US, T1_US, T2_US, T3_US, T_TOTAL_US, NOTE)      \
    do {                                                                             \
      ::flir::CsvSink::instance().write_timeline(                                    \
          (THREAD),                                                                  \
          static_cast<std::uint64_t>(SEQ),                                           \
          static_cast<std::uint64_t>(T0_US),                                         \
          static_cast<std::uint64_t>(T1_US),                                         \
          static_cast<std::uint64_t>(T2_US),                                         \
          static_cast<std::uint64_t>(T3_US),                                         \
          static_cast<std::uint64_t>(T_TOTAL_US),                                    \
          (NOTE));                                                                   \
    } while(0)

#else

  #define CSV_LOG_TL(THREAD, SEQ, T0_US, T1_US, T2_US, T3_US, T_TOTAL_US, NOTE)      \
    do {                                                                             \
      (void)(THREAD); (void)(SEQ); (void)(T0_US); (void)(T1_US);                     \
      (void)(T2_US); (void)(T3_US); (void)(T_TOTAL_US); (void)(NOTE);                \
    } while(0)

#endif
