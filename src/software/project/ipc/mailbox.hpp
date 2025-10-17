#pragma once
#include <optional>
#include <cstdint>

namespace flir {

// SPSC 템플릿(헤더온리). 여기선 인터페이스만 스켈레톤으로 제공.
// 실제 링버퍼 구현은 추후 채우기.
template <typename T>
class SpscMailbox {
public:
    // 생산자: 다른 스레드(캡처/네트)에서 호출 → 소비자 깨워야 함
    void push(const T& item) {
        // TODO: 고정 크기 링버퍼에 item 기록 + seq 증가
        (void)item;
    }

    // 소비자: run() 루프에서 새 데이터가 있는지 시퀀스로 판단
    bool has_new(uint32_t last_seen) const {
        // TODO: latest_seq_ > last_seen?
        (void)last_seen; return false;
    }

    // 소비자: 새 항목 pop (없으면 std::nullopt)
    std::optional<T> exchange(std::nullptr_t) {
        // TODO: head에서 하나 pop하여 반환
        return std::nullopt;
    }

    // 소비자: 최신 시퀀스 조회
    uint32_t latest_seq() const {
        // TODO
        return 0;
    }

private:
    // TODO: 링버퍼, head/tail, latest_seq_ 등 상태
};

} // namespace flir