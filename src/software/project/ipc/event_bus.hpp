#pragma once
#include "ipc_types.hpp"
#include "mailbox.hpp"
#include "wake.hpp"

namespace flir {

// 퍼블리시–서브스크라이브: push 시 내부에서 구독자 inbox로 라우팅 + 깨우기
struct IEventBus {
    virtual ~IEventBus() = default;

    // 구독 등록: 해당 토픽 이벤트가 오면 inbox로 전달 + wake.signal()
    virtual void subscribe(Topic topic, SpscMailbox<Event>* inbox, WakeHandle* wake) = 0;

    // 구독 해제
    virtual void unsubscribe(SpscMailbox<Event>* inbox) = 0;

    // 발행: 토픽 기반으로 구독자들에게 이벤트 분배
    virtual void push(const Event& e, Topic topic) = 0;
};

} // namespace flir
