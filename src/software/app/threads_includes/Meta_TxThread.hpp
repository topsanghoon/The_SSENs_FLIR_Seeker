#pragma once
#include <atomic>
#include <thread>
#include <cstdint>
#include <memory>                  // unique_ptr
#include <sys/socket.h>            // sockaddr, socklen_t
#include "ipc/ipc_types.hpp"
#include "ipc/mailbox.hpp"
#include "ipc/event_bus.hpp"

namespace flir {

// 메타 패킷 DTO (네트워크 전송 구조체: 프로토콜에 맞춰 확장)
struct MetaTrackPacket { uint32_t seq; uint64_t ts; float x,y,w,h; float score; };
struct MetaArucoPacket { uint32_t seq; uint64_t ts; int id; float x,y,w,h; };
struct MetaCtrlPacket  { uint32_t seq; uint64_t ts; int state_or_cmd; };
struct MetaHBPacket    { uint64_t ts; /* 요약 필드 */ };

// 최신값 캐시(하트비트용)
struct LatestTrack { cv::Rect2f box{}; float score=0.f; uint64_t ts=0; uint32_t frame_seq=0; };
struct LatestAruco { int id=-1; cv::Rect2f box{}; uint64_t ts=0; };
struct LatestCtrl  { int state=0; int last_cmd=0; uint64_t ts=0; };

struct MetaTxConfig {
    int  hb_period_ms    = 200;
    uint16_t local_port  = 0;                 // ★ 송신만이면 0 권장(바인드 생략)
    char remote_ip[64]   = "192.168.2.191"; // ★ WPF PC IP 기본값
    uint16_t remote_port = 5001;              // 필수
    int  sndbuf_bytes    = 256*1024;          // 선택
};

// EPOLL로 기다릴 fd 핸들 모음(주입 또는 내부 생성)
struct MetaFds {
    int epfd      = -1; // epoll fd
    int efd       = -1; // eventfd (EVT_BUS에서 깨움)
    int tfd       = -1; // timerfd (하트비트/리샘플)
    int sock_meta = -1; // 단일 UDP 소켓(모든 메타 전송)
};

class Meta_TxThread {
public:
    Meta_TxThread(IEventBus& bus,            // 이벤트 버스(구독)
                  MetaTxConfig cfg = MetaTxConfig{},
                  MetaFds fds = {});         // fds 직접 주입 가능(없으면 내부 생성)

    // 수명 제어
    void start();
    void stop();
    void join();

    // (테스트/주입용) 단일 메타 전송 목적지 설정
    void set_meta_target(const struct sockaddr* sa, socklen_t slen);

private:
    // 협력자
    IEventBus&               bus_;
    SpscMailbox<Event>       inbox_;      // 버스→나(메타)의 구독용 inbox
    MetaTxConfig             cfg_;
    MetaFds                  fds_;

    // 리소스 소유 여부(외부 주입과 구분)
    bool own_epfd_{false};
    bool own_efd_{false};
    bool own_tfd_{false};
    bool own_sock_{false};

    // 상태
    std::thread              th_;
    std::atomic<bool>        running_{false};
    LatestTrack              last_trk_{};
    LatestAruco              last_aru_{};
    LatestCtrl               last_ctl_{};
    uint64_t                 last_sent_ns_ = 0;

    // 네트워크 목적지(단일)
    struct sockaddr_storage  sa_meta_{};
    socklen_t                sl_meta_ = 0;

    // 버스 깨우기 핸들(인스턴스 멤버로 보유)
    std::unique_ptr<WakeHandle> wake_;

    // 내부 동작
    void run(); // epoll 루프 본체

    // epoll 이벤트 핸들러
    void on_eventfd_ready();   // EVT_BUS 신호 → inbox drain → 즉시 send
    void on_timerfd_ready();   // 주기 하트비트/리샘플

    // 송신 빌더
    MetaTrackPacket make_meta_track() const;
    MetaArucoPacket make_meta_aruco() const;
    MetaCtrlPacket  make_meta_ctrl () const;
    MetaHBPacket    make_meta_hb   () const;

    // 송신 (모두 같은 소켓/목적지 사용)
    void send_track(const MetaTrackPacket& p);
    void send_aruco(const MetaArucoPacket& p);
    void send_ctrl (const MetaCtrlPacket&  p);
    void send_hb   (const MetaHBPacket&    p);
};

} // namespace flir
