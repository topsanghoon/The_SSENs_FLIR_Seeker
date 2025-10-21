#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <thread>
#include <vector>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <opencv2/core.hpp>

#include "threads_includes/Meta_TxThread.hpp"
#include "ipc/event_bus.hpp"
#include "ipc/ipc_types.hpp"
#include "ipc/mailbox.hpp"

using namespace std::chrono_literals;

static std::atomic<bool> g_run{true};
static void on_sigint(int){ g_run.store(false); }

// ---- LocalEventBus: push만 구현하면 OK ----
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
            auto ev_copy = e;
            s.mb->push(std::move(ev_copy));     // ★ 프로듀서 API
            if (s.wh) s.wh->signal();
            break;
        }
    }
private:
    std::vector<Sub> subs_;
};

static bool make_sockaddr_ipv4(const char* ip, uint16_t port, sockaddr_storage& ss, socklen_t& sl) {
    sockaddr_in sa{}; sa.sin_family = AF_INET; sa.sin_port = htons(port);
    if (::inet_pton(AF_INET, ip, &sa.sin_addr) != 1) return false;
    std::memset(&ss, 0, sizeof(ss));
    std::memcpy(&ss, &sa, sizeof(sa));
    sl = sizeof(sa);
    return true;
}

int main(int argc, char** argv) {
    std::signal(SIGINT, on_sigint);
    if (argc < 3) {
        std::cerr << "사용법: " << argv[0] << " <WIN_IPv4> <PORT>\n";
        return 1;
    }
    const char* target_ip = argv[1];
    uint16_t target_port = static_cast<uint16_t>(std::stoi(argv[2]));

    int sock_meta = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_meta < 0) { perror("socket"); return 2; }

    sockaddr_storage sa{}; socklen_t sl = 0;
    if (!make_sockaddr_ipv4(target_ip, target_port, sa, sl)) {
        std::cerr << "IP 주소 파싱 실패\n"; return 3;
    }

    LocalEventBus bus;
    flir::MetaTxConfig cfg{}; cfg.hb_period_ms = 500;
    flir::MetaFds fds{}; fds.sock_meta = sock_meta;

    flir::Meta_TxThread tx(bus, cfg, fds);
    tx.set_meta_target(reinterpret_cast<sockaddr*>(&sa), sl);
    tx.start();

    std::cout << "[meta_txthread_test] start → " << target_ip << ":" << target_port << "\n";

    uint32_t seq = 0;
    uint64_t t0_ns = []{
        using namespace std::chrono;
        return duration_cast<nanoseconds>(steady_clock::now().time_since_epoch()).count();
    }();

    while (g_run.load()) {
        // Track
        {
            flir::TrackEvent tev{};
            tev.frame_seq = ++seq;
            tev.ts = t0_ns + seq * 20'000'000ULL;
            float x = 10.f + (seq % 100);
            float y = 20.f + ((seq*3) % 60);
            tev.box = cv::Rect2f{x, y, 40.f, 30.f};
            tev.score = 0.7f + 0.3f * ((seq % 10) / 10.0f);

            flir::Event e{};
            e.type = flir::EventType::Track;
            e.payload = tev;
            bus.push(e, flir::Topic::Tracking);

            std::cout << "[push] Track seq=" << tev.frame_seq << " ts=" << tev.ts << "\n";
        }

        // ArUco
        if ((seq % 15) == 0) {
            flir::ArucoEvent a{};
            a.ts = t0_ns + seq * 20'000'000ULL;
            a.id = (seq/15) % 5;
            a.box = cv::Rect2f{80.f, 60.f, 24.f, 24.f};

            flir::Event e{};
            e.type = flir::EventType::Aruco;
            e.payload = a;
            bus.push(e, flir::Topic::Aruco);

            std::cout << "[push] Aruco id=" << a.id << " ts=" << a.ts << "\n";
        }

        // MetaCtrl
        if ((seq % 30) == 0) {
            flir::MetaCtrlEvent c{};
            c.ts = t0_ns + seq * 20'000'000ULL;
            c.cmd = (seq/30) % 3;

            flir::Event e{};
            e.type = flir::EventType::MetaCtrl;
            e.payload = c;
            bus.push(e, flir::Topic::Control);

            std::cout << "[push] Ctrl cmd=" << c.cmd << " ts=" << c.ts << "\n";
        }

        std::this_thread::sleep_for(100ms);
    }

    tx.stop();
    tx.join();
    ::close(sock_meta);
    return 0;
}
