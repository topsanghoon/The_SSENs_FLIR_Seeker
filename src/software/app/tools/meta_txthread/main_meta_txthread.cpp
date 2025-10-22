// main_meta_txthread.cpp (헤더 기본값 100% 사용)
#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <thread>
#include <iostream>

#include <opencv2/core.hpp>

#include "threads_includes/Meta_TxThread.hpp"
#include "ipc/event_bus.hpp"
#include "ipc/ipc_types.hpp"
#include "ipc/mailbox.hpp"

using namespace std::chrono_literals;

static std::atomic<bool> g_run{true};
static void on_sigint(int){ g_run.store(false); }

// 최소 EventBus: push만
class LocalEventBus final : public flir::IEventBus {
public:
    struct Sub { flir::Topic topic; flir::SpscMailbox<flir::Event>* mb{}; flir::WakeHandle* wh{}; };
    void subscribe(flir::Topic t, flir::SpscMailbox<flir::Event>* inbox, flir::WakeHandle* wake) override {
        subs_.push_back({t, inbox, wake});
    }
    void unsubscribe(flir::SpscMailbox<flir::Event>* inbox) override {
        subs_.erase(std::remove_if(subs_.begin(), subs_.end(),
                      [&](const Sub& s){ return s.mb == inbox; }), subs_.end());
    }
    void push(const flir::Event& e, flir::Topic t) override {
        for (auto& s : subs_) {
            if (s.topic != t) continue;
            auto copy = e; s.mb->push(std::move(copy));
            if (s.wh) s.wh->signal();
            break;
        }
    }
private:
    std::vector<Sub> subs_;
};

int main() {
    std::signal(SIGINT, on_sigint);

    LocalEventBus bus;

    // ★ 헤더 기본값만 사용
    flir::MetaTxConfig cfg{};   // remote_ip, remote_port, local_port 전부 헤더의 default
    flir::MetaFds fds{};        // sock_meta=-1 → 내부 생성 사용

    flir::Meta_TxThread tx(bus, cfg, fds);
    tx.start();

    std::cout << "[meta_txthread_test] start (using MetaTxConfig defaults)\n"
              << "Ctrl+C로 종료. HB + 샘플 Track/Aruco/Ctrl 전송.\n";

    uint32_t seq = 0;
    const uint64_t t0_ns = []{
        using namespace std::chrono;
        return duration_cast<nanoseconds>(steady_clock::now().time_since_epoch()).count();
    }();

    while (g_run.load()) {
        // Track
        {
            flir::TrackEvent tev{};
            tev.frame_seq = ++seq;
            tev.ts = t0_ns + seq * 20'000'000ULL; // 20ms 간격
            float x = 10.f + (seq % 100);
            float y = 20.f + ((seq*3) % 60);
            tev.box   = cv::Rect2f{x, y, 40.f, 30.f};
            tev.score = 0.7f + 0.3f * ((seq % 10) / 10.0f);

            flir::Event e{};
            e.type = flir::EventType::Track;
            e.payload = tev;
            bus.push(e, flir::Topic::Tracking);
        }

        // ArUco (가끔)
        if ((seq % 15) == 0) {
            flir::ArucoEvent a{};
            a.ts = t0_ns + seq * 20'000'000ULL;
            a.id = (seq/15) % 5;
            a.box = cv::Rect2f{80.f, 60.f, 24.f, 24.f};

            flir::Event e{};
            e.type = flir::EventType::Aruco;
            e.payload = a;
            bus.push(e, flir::Topic::Aruco);
        }

        // MetaCtrl (가끔)
        if ((seq % 30) == 0) {
            flir::MetaCtrlEvent c{};
            c.ts  = t0_ns + seq * 20'000'000ULL;
            c.cmd = (seq/30) % 3;
            std::cout << c.ts << "\n";

            flir::Event e{};
            e.type = flir::EventType::MetaCtrl;
            e.payload = c;
            bus.push(e, flir::Topic::Control);
        }

        std::this_thread::sleep_for(100ms);
    }

    tx.stop();
    tx.join();
    return 0;
}
