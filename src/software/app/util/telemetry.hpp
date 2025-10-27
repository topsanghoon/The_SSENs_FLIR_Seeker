#pragma once

// ===== 컴파일 타임 스위치 =====
//  - FLIR_LOG_DISABLE    : printf/stream 로그 완전 제거(이미 사용중)
//  - FLIR_CSV_DISABLE    : CSV 저장 완전 제거(새로 추가)
//  - FLIR_PROFILE_FORCE_DISABLE : 강제로 시간계측도 제거(옵션)

#if defined(FLIR_LOG_DISABLE)
#  define FLIR_LOG_ENABLED 0
#else
#  define FLIR_LOG_ENABLED 1
#endif

#if defined(FLIR_CSV_DISABLE)
#  define FLIR_CSV_ENABLED 0
#else
#  define FLIR_CSV_ENABLED 1
#endif

#if (FLIR_LOG_ENABLED || FLIR_CSV_ENABLED) && !defined(FLIR_PROFILE_FORCE_DISABLE)
#  define FLIR_PROFILE_ENABLED 1
#else
#  define FLIR_PROFILE_ENABLED 0
#endif

// CSV 구문을 간단히 감싸는 매크로
#if FLIR_CSV_ENABLED
#  define CSV_DO(stmt) do { stmt; } while(0)
#else
#  define CSV_DO(stmt) do { } while(0)
#endif
