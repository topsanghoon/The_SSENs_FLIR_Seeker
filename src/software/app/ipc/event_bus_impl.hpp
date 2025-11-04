#pragma once
#include <unordered_map>
#include <vector>
#include <mutex>
#include <algorithm>
#include "ipc/event_bus.hpp"

namespace flir {

class EventBus : public IEventBus {
public:
    void subscribe(Topic topic, SpscMailbox<Event>* inbox, WakeHandle* wake) override;
    void unsubscribe(SpscMailbox<Event>* inbox) override;
    void push(const Event& e, Topic topic) override;

private:
    struct Sub { SpscMailbox<Event>* q; WakeHandle* wake; };

    std::unordered_map<Topic, std::vector<Sub>> subs_;
    std::mutex m_;
};

} // namespace flir
