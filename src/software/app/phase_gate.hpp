// phase_gate.hpp
#pragma once
#include "guidance_mode.hpp"

namespace flir {

// EO는 중기(Midcourse)에서만 활성
inline bool eo_enabled() {
    return GuidanceState::phase().load(std::memory_order_relaxed) == GuidancePhase::Midcourse;
}

// IR은 종말(Terminal)에서만 활성
inline bool ir_enabled() {
    return GuidanceState::phase().load(std::memory_order_relaxed) == GuidancePhase::Terminal;
}

} // namespace flir
