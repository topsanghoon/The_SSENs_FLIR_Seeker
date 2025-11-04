#include "ipc/event_bus_impl.hpp"

namespace flir {
void EventBus::push(const Event& e, Topic topic) {
    std::vector<Sub> targets;
    { // 스냅샷
        std::lock_guard<std::mutex> lk(m_);
        auto it = subs_.find(topic);
        if (it == subs_.end()) return;
        targets = it->second; // 복사
    }
    // 락 없이 분배 + 깨우기
    for (auto& s : targets) {
        s.q->push(e);
        if (s.wake) s.wake->signal();
    }
}

void EventBus::subscribe(Topic topic, SpscMailbox<Event>* inbox, WakeHandle* wake) {
    std::lock_guard<std::mutex> lk(m_);
    subs_[topic].push_back({inbox, wake});
}

void EventBus::unsubscribe(SpscMailbox<Event>* inbox) {
    std::lock_guard<std::mutex> lk(m_);
    for (auto it = subs_.begin(); it != subs_.end(); ) {
        auto& vec = it->second;
        vec.erase(std::remove_if(vec.begin(), vec.end(),
                  [&](const Sub& s){ return s.q == inbox; }), vec.end());
        if (vec.empty()) it = subs_.erase(it); else ++it;
    }
}


} // namespace flir
