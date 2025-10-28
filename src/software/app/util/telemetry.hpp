// app/components/util/telemetry.hpp
#pragma once

// CSV를 켜고/끄는 스위치 (원하면 CMake에서 -DFLIR_CSV_ENABLED=0/1로 오버라이드)
#ifndef FLIR_CSV_ENABLED
#define FLIR_CSV_ENABLED 1
#endif

// common_log는 선택(없어도 무방)
#include "util/common_log.hpp"

// CsvSink 선언/정의
#include "util/csv_sink.hpp"

// ─────────────────────────────────────────────────────────────
// 통합 CSV 로깅 매크로
// 사용형: CSV_LOG_SIMPLE("Thread.Tag", "EVENT", seq, t0, t1, t2, total, "note")
//  - 첫 번째 인자: thread/tag
//  - 두 번째 인자: level/event (문자열)
// ─────────────────────────────────────────────────────────────
#if FLIR_CSV_ENABLED
  #define CSV_LOG_SIMPLE(THREAD, LEVEL, SEQ, T0, T1, T2, TOTAL, NOTE)                        \
    do {                                                                                     \
      ::flir::CsvSink::instance().write_simple(                                              \
          /*level*/ (LEVEL),                                                                 \
          /*thread*/ (THREAD),                                                               \
          /*seq*/ static_cast<unsigned long>(SEQ),                                           \
          /*t0*/  static_cast<double>(T0),                                                   \
          /*t1*/  static_cast<double>(T1),                                                   \
          /*t2*/  static_cast<double>(T2),                                                   \
          /*tot*/ static_cast<double>(TOTAL),                                                \
          /*note*/ (NOTE));                                                                  \
    } while(0)
#else
  #define CSV_LOG_SIMPLE(THREAD, LEVEL, SEQ, T0, T1, T2, TOTAL, NOTE) do { (void)0; } while(0)
#endif
