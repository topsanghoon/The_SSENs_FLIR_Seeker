#pragma once
#include <atomic>
#include <cstdint>

namespace flir {

enum class GuidancePhase : uint8_t { Midcourse = 0, Terminal = 1 };

struct GuidanceState {
  static std::atomic<GuidancePhase>& phase() {
    static std::atomic<GuidancePhase> g{GuidancePhase::Midcourse};
    return g;
  }
};

} // namespace flir
