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

// Capture (fan-out 내장)
#include "threads_includes/IR_CaptureThread.hpp"
#include "threads_includes/EO_CaptureThread.hpp"

// Tx
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
#include "components/includes/ActuatorPort.hpp"

using namespace flir;
using namespace std::chrono_literals;

static std::atomic<bool> g_quit{false};
static void sig_handler(int s){ std::cout << "\n[SIG] " << s << " → quit\n"; g_quit.store(true); }

struct DummyWakeTx  : WakeHandle { void signal() override {} };

int main(int, char**) {
    std::signal(SIGINT,  sig_handler);
    std::signal(SIGTERM, sig_handler);

    auto app = std::make_shared<AppConfig>();

    // ====== Config ======
    app->guidance.default_phase = GuidancePhase::Midcourse;
    GuidanceState::phase().store(app->guidance.default_phase);

    app->eo_tx.frame = {320,240};
    app->eo_tx.fps = 15;
    app->eo_tx.jpeg_quality = 30;
    app->eo_tx.dst = {"192.168.0.19", 5003};

    app->ir_tx.frame = {80,60};     // Lepton 2.5 기본
    app->ir_tx.fps   = 9;
    app->ir_tx.bitDepth = 16;
    app->ir_tx.dst = {"192.168.0.19", 5002};

    app->meta_tx.dst        = {"192.168.0.19", 5001};
    app->meta_tx.local_port = 0;
    app->meta_tx.hb_period_ms = 1000;

    app->net_rx.port          = 5000;
    app->net_rx.timeout_ms    = 50;
    app->net_rx.buffer_size   = 2048;
    app->net_rx.click_box_size= 40.0f;

    app->guidance.terminal_marker_id = 3;
    app->guidance.min_bbox_w = 160;
    app->guidance.min_bbox_h = 160;
    app->guidance.min_big_frames = 5;
    app->guidance.lost_timeout_ms = 300;
    app->guidance.min_bbox_frac = 0.0f;

    // ====== IPC ======
    EventBus bus;

    // fan-out 내장이라 cap 큐는 없음. 소비자별 큐만 준비
    SpscMailbox<std::shared_ptr<IRFrameHandle>> mb_ir_tx(8);
    SpscMailbox<std::shared_ptr<IRFrameHandle>> mb_ir_trk(8);

    SpscMailbox<std::shared_ptr<EOFrameHandle>> mb_eo_tx(8);
    SpscMailbox<std::shared_ptr<EOFrameHandle>> mb_eo_aru(8);

    SpscMailbox<UserCmd>         mb_click(64);
    SpscMailbox<SelfDestructCmd> mb_sd(4);

    // ====== Threads ======
    // Tx 먼저 준비 (EO는 wake를 캡처에 넘김)
    DummyWakeTx dummy_wake_tx;
    IR_TxThread ir_tx("IR_Tx", app, mb_ir_tx, dummy_wake_tx);

    EO_TxThread eo_tx("EO_Tx", app, mb_eo_tx, dummy_wake_tx);
    auto eo_wake_for_cap = eo_tx.create_wake_handle();

    // Capture (fan-out 내장)
    IR_CaptureThread ir_cap("IR_Cap", mb_ir_tx, mb_ir_trk, app);
    EO_CaptureThread eo_cap("EO_Cap", mb_eo_tx, mb_eo_aru, std::move(eo_wake_for_cap), app);

    // 분석
    IR_Preprocessor  ir_pre(1.0f/16383.0f, 0.0f);
    IR_Tracker_MOSSE ir_mosse;
    IRTrackConfig    trk_cfg{}; trk_cfg.user_req_threshold = 15;
    IR_TrackThread   ir_track(mb_ir_trk, mb_click, ir_mosse, ir_pre, bus, trk_cfg);

    EO_ArucoDetector_OpenCV aruco(cv::aruco::DICT_4X4_50);
    struct SimpleArucoPreproc : IArucoPreprocessor {
        void run(const EOFrameHandle& in, cv::Mat& gray) override {
            const FrameBGR8& f = *in.p;
            cv::Mat src(f.height, f.width, CV_8UC3, (void*)f.data, f.step);
            cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);
        }
    } arupre;
    EO_ArUcoThread eo_aru(mb_eo_aru, arupre, aruco, bus);

    // 제어
    ControlThread::Config ccfg;
    ccfg.period_ms = 20;
    ccfg.guidance  = app->guidance;
    ccfg.eo_w      = app->eo_tx.frame.width;
    ccfg.eo_h      = app->eo_tx.frame.height;

    TargetFusion  fusion;
    ControllerLR  controller(app->eo_tx.frame.width, 4.0f);
    UART_ActuatorPort act("/dev/ttyUL0", B115200);
    ControlThread control(bus, mb_sd, fusion, controller, act, ccfg);

    // 메타/네트
    Meta_TxThread meta_tx(bus, app);
    Net_RxThread  net_rx("Net_Rx", app, mb_click);

    // ====== Start ======
    ir_cap.start();
    eo_cap.start();

    ir_tx.start();
    eo_tx.start();

    ir_track.start();
    eo_aru.start();

    meta_tx.start();
    net_rx.start();

    control.start();

    std::cout << "[INFO] Midcourse start. EO active, IR idle. Ctrl+C to exit.\n";

    while (!g_quit.load()) std::this_thread::sleep_for(50ms);

    // ====== Stop ======
    control.stop();  control.join();
    net_rx.stop();   net_rx.join();
    meta_tx.stop();  meta_tx.join();
    ir_track.stop(); ir_track.join();
    eo_aru.stop();   eo_aru.join();
    ir_tx.stop();    ir_tx.join();
    eo_tx.stop();    eo_tx.join();
    ir_cap.stop();   ir_cap.join();
    eo_cap.stop();   eo_cap.join();

    return 0;
}
