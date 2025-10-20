#pragma once

namespace flir {

// 스레드를 깨우는 추상 핸들. 구현은 condvar, eventfd, pipe 등으로 래핑.
struct WakeHandle {
    virtual ~WakeHandle() = default;
    virtual void signal() = 0; // 잠든 run()을 깨움
};

} // namespace flir
