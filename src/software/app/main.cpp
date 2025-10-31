// main.cpp
#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <memory>
#include <thread>

#include <opencv2/opencv.hpp>

#include "main_config.hpp"
#include "guidance_mode.hpp"

#include "ipc/event_bus_impl.hpp"
#include "ipc/mailbox.hpp"
#include "ipc/wake.hpp"

// Capture
#include "threads_includes/IR_CaptureThread.hpp"
#include "threads_includes/EO_CaptureThread.hpp"

// Tx (항상 포함)
#include "threads_includes/IR_TxThread.hpp"
#include "threads_includes/EO_TxThread.hpp"

// 분석/제어
#include "threads_includes/IR_TrackThread.hpp"
#include "threads_includes/EO_ArUcoThread.hpp"
#include "threads_includes/Meta_TxThread.hpp"
#include "threads_includes/Net_RxThread.hpp"
#include "threads_includes/ControlThread.hpp"

#include "components/includes/IR_Preprocessor.hpp"
#include "components/includes/IR_Tracker_MOSSE.hpp"
#include "components/includes/EO_ArucoDetector_OpenCV.hpp"
#include "components/includes/TargetFusion.hpp"
#include "components/includes/IController.hpp"
#include "components/includes/ActuatorPort.hpp"   // ★ UART_ActuatorPort 선언

using namespace flir;
using namespace std::chrono_literals;

// ---------- 유틸 ----------
static std::atomic<bool> g_quit{false};
static void sig_handler(int s){ std::cout << "\n[SIG] " << s << " → quit\n"; g_quit.store(true); }

// WakeHandle 더미(캡처/Tx용)
struct DummyWakeCap : WakeHandle { void signal() override {} };
struct DummyWakeTx  : WakeHandle { void signal() override {} };

// 아주 단순한 컨트롤러(실제 것으로 교체 가능)
struct NopController final : IController {
    CtrlCmd solve(const TargetFusion&) override { return CtrlCmd{}; } // no-op
};

int main(int, char**) {
    std::signal(SIGINT,  sig_handler);
    std::signal(SIGTERM, sig_handler);

    // ================= AppConfig =================
    auto app = std::make_shared<AppConfig>();

    // 시작 단계: 중기유도(EO만 활성)
    app->guidance.default_phase = GuidancePhase::Midcourse;
    GuidanceState::phase().store(app->guidance.default_phase);

    // EO(웹캠)
    app->eo_tx.frame = {640,480};
    app->eo_tx.fps = 15;
    app->eo_tx.jpeg_quality = 30;
    app->eo_tx.dst = {"192.168.2.191", 5003};

    // IR(Lepton) — 초기 2.5(80×60). 3.0 쓰려면 {160,120}로 변경
    app->ir_tx.frame = {80,60};
    app->ir_tx.fps   = 9;
    app->ir_tx.bitDepth = 16;
    app->ir_tx.dst = {"192.168.2.191", 5002};

    // Meta / Net
    app->meta_tx.dst        = {"192.168.2.191", 5001};
    app->meta_tx.local_port = 0;
    app->meta_tx.hb_period_ms = 1000;

    app->net_rx.port          = 5000;
    app->net_rx.timeout_ms    = 50;
    app->net_rx.buffer_size   = 2048;
    app->net_rx.click_box_size= 40.0f;

    // 전환 파라미터
    app->guidance.terminal_marker_id = 3;
    app->guidance.min_bbox_w = 160;
    app->guidance.min_bbox_h = 160;
    app->guidance.min_big_frames = 5;
    app->guidance.lost_timeout_ms = 300;
    app->guidance.min_bbox_frac = 0.0f;

    // ================= IPC =================
    EventBus bus;

    // Capture 출력(단일 생산자) → Fan-out을 위한 펌프 입력 큐
    SpscMailbox<std::shared_ptr<IRFrameHandle>> mb_ir_cap(8);
    SpscMailbox<std::shared_ptr<EOFrameHandle>> mb_eo_cap(8);

    // Fan-out 목적지: Tx와 분석(트래킹/아루코) 각각 전용 큐
    SpscMailbox<std::shared_ptr<IRFrameHandle>> mb_ir_tx(8);
    SpscMailbox<std::shared_ptr<IRFrameHandle>> mb_ir_trk(8);

    SpscMailbox<std::shared_ptr<EOFrameHandle>> mb_eo_tx(8);
    SpscMailbox<std::shared_ptr<EOFrameHandle>> mb_eo_aru(8);

    // 클릭/자폭
    SpscMailbox<UserCmd>         mb_click(64);
    SpscMailbox<SelfDestructCmd> mb_sd(4);

    // ================= Threads =================
    // Capture
    IR_CaptureThread ir_cap("IR_Cap", mb_ir_cap, app);
    EO_CaptureThread eo_cap("EO_Cap", mb_eo_cap, std::make_unique<DummyWakeCap>(), app);

    // Tx (GStreamer 기반) — 팬아웃된 큐를 소비
    DummyWakeTx dummy_wake_tx;
    IR_TxThread ir_tx("IR_Tx", app, mb_ir_tx, dummy_wake_tx);
    EO_TxThread eo_tx("EO_Tx", app, mb_eo_tx, dummy_wake_tx);

    // IR Tracking (복제본 큐 소비)
    IR_Preprocessor ir_pre(1.0f/16383.0f, 0.0f);
    IR_Tracker_MOSSE ir_mosse;
    IRTrackConfig trk_cfg{}; trk_cfg.user_req_threshold = 15;
    IR_TrackThread ir_track(mb_ir_trk, mb_click, ir_mosse, ir_pre, bus, trk_cfg);

    // EO ArUco (복제본 큐 소비)
    EO_ArucoDetector_OpenCV aruco(cv::aruco::DICT_4X4_50);
    struct SimpleArucoPreproc : IArucoPreprocessor {
        void run(const EOFrameHandle& in, cv::Mat& gray) override {
            const FrameBGR8& f = *in.p;
            cv::Mat src(f.height, f.width, CV_8UC3, (void*)f.data, f.step);
            cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);
        }
    } arupre;
    EO_ArUcoThread eo_aru(mb_eo_aru, arupre, aruco, bus);

    // Control (전환 FSM + 9001 송신)
    ControlThread::Config ccfg;
    ccfg.period_ms = 20;
    ccfg.guidance  = app->guidance;
    ccfg.eo_w      = app->eo_tx.frame.width;
    ccfg.eo_h      = app->eo_tx.frame.height;

    TargetFusion   fusion;
    NopController  controller;

    // ★ UART 포트 생성 (네 환경에 맞춰 경로/보율 바꿔)
    //    예) "/dev/ttyPS1" (Zynq UART), "/dev/ttyUSB0" (USB-Serial)
    UART_ActuatorPort act("/dev/ttyUSB0", B115200);

    ControlThread control(bus, mb_sd, fusion, controller, act, ccfg);

    // Meta / Net — ★ 실제 객체를 여기서 생성 (식별자 미정의 방지)
    Meta_TxThread meta_tx(bus, app);
    Net_RxThread  net_rx("Net_Rx", app, mb_click);

    // ================= Fan-out 펌프 (capture→[tx, 분석] 복제) =================
    std::atomic<bool> pump_run{true};
    std::thread pump_ir([&]{
        while (pump_run.load(std::memory_order_relaxed)) {
            if (auto h = mb_ir_cap.exchange(nullptr)) {
                mb_ir_tx.push(*h);
                mb_ir_trk.push(*h);
            } else {
                std::this_thread::sleep_for(1ms);
            }
        }
    });
    std::thread pump_eo([&]{
        while (pump_run.load(std::memory_order_relaxed)) {
            if (auto h = mb_eo_cap.exchange(nullptr)) {
                mb_eo_tx.push(*h);
                mb_eo_aru.push(*h);
            } else {
                std::this_thread::sleep_for(1ms);
            }
        }
    });

    // ================= Start =================
    ir_cap.start();
    eo_cap.start();

    ir_tx.start();
    eo_tx.start();

    ir_track.start();
    eo_aru.start();

    meta_tx.start();             // ★ 이제 식별자 존재
    net_rx.start();

    control.start();

    std::cout << "[INFO] Midcourse start. EO active, IR idle. Ctrl+C to exit.\n";

    // ================= Loop =================
    while (!g_quit.load()) std::this_thread::sleep_for(50ms);

    // ================= Stop =================
    control.stop();  control.join();
    net_rx.stop();   net_rx.join();
    meta_tx.stop();  meta_tx.join();
    ir_track.stop(); ir_track.join();
    eo_aru.stop();   eo_aru.join();
    ir_tx.stop();    ir_tx.join();
    eo_tx.stop();    eo_tx.join();
    ir_cap.stop();   ir_cap.join();
    eo_cap.stop();   eo_cap.join();

    pump_run.store(false);
    pump_ir.join();
    pump_eo.join();

    return 0;
}
