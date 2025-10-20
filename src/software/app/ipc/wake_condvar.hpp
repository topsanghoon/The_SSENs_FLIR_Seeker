// src/software/app/ipc/wake_condvar.hpp
#pragma once
#include <condition_variable>
#include "ipc/wake.hpp"   // ← 여기!

namespace flir {
class WakeHandleCondVar : public WakeHandle {
public:
    explicit WakeHandleCondVar(std::condition_variable& cv) : cv_(cv) {}
    void signal() override { cv_.notify_one(); }
private:
    std::condition_variable& cv_;
};
} // namespace flir
