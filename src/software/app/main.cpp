#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <memory>
#include <thread>

#include <sys/auxv.h>
#include <asm/hwcap.h>
#include <stdio.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/utility.hpp>

#include "main_config.hpp"
#include "guidance_mode.hpp"

#include "ipc/event_bus_impl.hpp"
#include "ipc/mailbox.hpp"
#include "ipc/wake.hpp"

// Capture
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
#include "components/includes/IR_Tracker_KCF.hpp"
#include "components/includes/IR_Tracker_CSRT.hpp"
#include "components/includes/EO_ArucoDetector_OpenCV.hpp"
#include "components/includes/TargetFusion.hpp"
#include "components/includes/IController.hpp"
#include "components/includes/ActuatorPort.hpp"

using namespace flir;
using namespace std::chrono_literals;

// ===== 종료 제어 =====
static std::atomic<bool> g_quit{false};
static void sig_handler(int s){ std::cout << "\n[SIG] " << s << " → quit\n"; g_quit.store(true); }

// 테스트용 더미 웨이크(실운용에선 불필요)
struct DummyWakeTx : WakeHandle { void signal() override {} };

int main(int, char**) {

    cv::setUseOptimized(true);
    cv::setNumThreads(1);
    // ─────────────────────────────────────────────────────────────
    // 0) 프로세스 레벨 초기화
    // ─────────────────────────────────────────────────────────────
    std::signal(SIGINT,  sig_handler);
    std::signal(SIGTERM, sig_handler);

    // unsigned long caps = getauxval(AT_HWCAP);
    // #ifdef HWCAP_NEON
    //     printf("[INFO] CPU HWCAP: 0x%lx (NEON %s)\n", caps, (caps & HWCAP_NEON) ? "YES" : "NO");
    // #endif

    // std::cout << cv::getBuildInformation() << std::endl;

    // bool neon = cv::checkHardwareSupport(CV_CPU_NEON);
    // std::cout << "[INFO] NEON support: " << (neon ? "YES" : "NO") << std::endl;*

    std::cout <<"[INFO] Application starting..." << std::endl;

    // 전역 설정
    auto app = std::make_shared<AppConfig>();
    {
        // 유도 단계(부팅 시점)
        app->guidance.default_phase = GuidancePhase::Midcourse;
        GuidanceState::phase().store(app->guidance.default_phase);

        // EO 송출
        app->eo_tx.frame = {320,240};
        app->eo_tx.fps = 30;
        app->eo_tx.jpeg_quality = 80;
        app->eo_tx.dst = {"192.168.0.250", 5003};

        // IR 송출 (Lepton 2.5)
        app->ir_tx.frame = {80,60};
        app->ir_tx.fps   = 9;
        app->ir_tx.bitDepth = 16;
        app->ir_tx.dst = {"192.168.0.250", 5002};

        // 메타/네트
        app->meta_tx.dst          = {"192.168.0.250", 5001};
        app->meta_tx.local_port   = 0;
        app->meta_tx.hb_period_ms = 1000;
        app->net_rx.port          = 5000;
        app->net_rx.timeout_ms    = 50;
        app->net_rx.buffer_size   = 2048;
        app->net_rx.click_box_size= 30.0f;

        // 유도 파라미터
        app->guidance.terminal_marker_id = 3;
        app->guidance.min_bbox_w = 120;
        app->guidance.min_bbox_h = 120;
        app->guidance.min_big_frames = 5;
        app->guidance.hold_big_ms = 300;
        app->guidance.min_bbox_frac = 0.0f;
    }

    // ─────────────────────────────────────────────────────────────
    // 1) IPC: 이벤트버스/메일박스
    // ─────────────────────────────────────────────────────────────
    EventBus bus;

    // IR: TX/Track 소비자 큐
    SpscMailbox<std::shared_ptr<IRFrameHandle>> mb_ir_tx(2);
    SpscMailbox<std::shared_ptr<IRFrameHandle>> mb_ir_trk(1);

    // EO: TX/Aruco 소비자 큐
    SpscMailbox<std::shared_ptr<EOFrameHandle>> mb_eo_tx(2);
    SpscMailbox<std::shared_ptr<EOFrameHandle>> mb_eo_aru(1);

    // 입력/자폭
    SpscMailbox<UserCmd>         mb_click(8);
    SpscMailbox<SelfDestructCmd> mb_sd(2);

    // ─────────────────────────────────────────────────────────────
    // 2) 스레드 구성: TX → Capture → Analysis 순으로 의존성 배치
    //    (TX가 웨이크 소유, Capture는 TX 깨우기; 분석은 onFrameArrived 경유)
    // ─────────────────────────────────────────────────────────────

    // --- EO TX ---
    DummyWakeTx dummy_wake_tx;
    EO_TxThread eo_tx("EO_Tx", app, mb_eo_tx, dummy_wake_tx);
    auto eo_wake_for_cap = eo_tx.create_wake_handle(); // 캡처가 TX를 깨움

    // --- EO Capture (캡처→TX로 push + TX wake; 캡처→Aruco는 sink 경유) ---
    EO_CaptureThread eo_cap("EO_Cap", mb_eo_tx, mb_eo_aru, std::move(eo_wake_for_cap), app);

    // --- IR TX ---
    IR_TxThread::GstConfig ir_gst{};
    ir_gst.width  = app->ir_tx.frame.width;
    ir_gst.height = app->ir_tx.frame.height;
    ir_gst.fps    = app->ir_tx.fps;
    ir_gst.pc_ip  = app->ir_tx.dst.ip;
    ir_gst.port   = app->ir_tx.dst.port;
    IR_TxThread ir_tx("IR_Tx", mb_ir_tx, ir_gst);
    auto ir_wake_for_cap = ir_tx.create_wake_handle(); // 캡처가 TX를 깨움

    // --- IR Capture (캡처→TX로 push + TX wake; 캡처→Track은 sink 경유) ---
    IRCaptureConfig ir_cap_cfg{};
    ir_cap_cfg.spi_device      = "/dev/spidev1.0";
    ir_cap_cfg.spi_speed       = 6'250'000;   // 안정성 우선(필요시 16MHz/20MHz로 조정)
    ir_cap_cfg.spi_delay_usecs = 0;
    ir_cap_cfg.fps             = app->ir_tx.fps;

    // ⬇️ 너의 코드베이스가 dual-mb 생성자를 제공한다면 아래 라인을 사용:
    IR_CaptureThread ir_cap("IR_Cap", mb_ir_tx, mb_ir_trk, std::move(ir_wake_for_cap), ir_cap_cfg);
    // 만약 단일-mb 생성자만 있다면:
    // IR_CaptureThread ir_cap("IR_Cap", mb_ir_tx, std::move(ir_wake_for_cap), ir_cap_cfg);
    // ir_cap.set_track_mailbox(&mb_ir_trk);

    // --- IR Tracking ---
    IR_Preprocessor  ir_pre(1.0f/16383.0f, 0.0f);
    IR_Tracker_MOSSE ir_mosse;
    IR_Tracker_KCF ir_kcf;
    IR_Tracker_CSRT ir_csrt;
    IRTrackConfig    trk_cfg{}; trk_cfg.user_req_threshold = 15;
    IR_TrackThread   ir_track(mb_ir_trk, mb_click, ir_kcf, ir_pre, bus, trk_cfg);

    // --- EO ArUco ---
    EO_ArucoDetector_OpenCV aruco(cv::aruco::DICT_4X4_50);
    struct SimpleArucoPreproc : IArucoPreprocessor {
        void run(const EOFrameHandle& in, cv::Mat& gray) override {
            const FrameBGR8& f = *in.p;
            cv::Mat src(f.height, f.width, CV_8UC3, (void*)f.data, f.step);
            cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);
        }
    } arupre;
    EO_ArUcoThread eo_aru(mb_eo_aru, arupre, aruco, bus);

    // --- Control / Meta / Net ---
    ControlThread::Config ccfg;
    ccfg.period_ms = 20;
    ccfg.guidance  = app->guidance;
    ccfg.eo_w      = app->eo_tx.frame.width;
    ccfg.eo_h      = app->eo_tx.frame.height;

    TargetFusion  fusion;
    ControllerLR controller(app->eo_tx.frame.width,  // EO 폭
                        app->ir_tx.frame.width,  // IR 폭
                        4.0f);                   // deadzone(px)
    UART_ActuatorPort act("/dev/ttyUL0", B115200);
    ControlThread control(bus, mb_sd, fusion, controller, act, ccfg);

    Meta_TxThread meta_tx(bus, app);
    Net_RxThread  net_rx("Net_Rx", app, mb_click, mb_sd, bus);

    // ─────────────────────────────────────────────────────────────
    // 3) “push+notify” 경로 확정: 캡처 → 분석 스레드로는 반드시 onFrameArrived 경유
    //    (여기서 notify가 발생하므로 소비자는 풀링 없이 깨어남)
    // ─────────────────────────────────────────────────────────────
    eo_cap.set_aru_sink([&](std::shared_ptr<EOFrameHandle> h){
        eo_aru.onFrameArrived(std::move(h));       // 내부에서 cv_.notify_one()
    });
    ir_cap.set_track_sink([&](std::shared_ptr<IRFrameHandle> h){
        ir_track.onFrameArrived(std::move(h));     // 내부에서 cv_.notify_one()
    });
    // ※ 메일박스로 직접 push만 하는 경로는 사용하지 않음(놓치면 영원히 안 깰 수 있음).
    //    꼭 사용해야 한다면 소비자에서 받은 WakeHandle.signal()까지 함께 호출할 것.

    control.set_on_shutdown([&]{ g_quit.store(true); });
    // ─────────────────────────────────────────────────────────────
    // 4) Start (소비자 → 프로듀서 → Tx 순으로 켜면 초기 드롭 최소화)
    // ─────────────────────────────────────────────────────────────
    ir_track.start();
    eo_aru.start();

    ir_cap.start();
    eo_cap.start();

    ir_tx.start();
    eo_tx.start();

    meta_tx.start();
    net_rx.start();
    
    control.start();

    std::cout << "[INFO] Midcourse start. Ctrl+C to exit.\n";
    while (!g_quit.load()) std::this_thread::sleep_for(50ms);

    // ─────────────────────────────────────────────────────────────
    // 5) Stop (권장 순서)
    control.stop();  control.join();
    net_rx.stop();   net_rx.join();
    meta_tx.stop();  meta_tx.join();

    // 분석 먼저 정지
    ir_track.stop(); ir_track.join();
    eo_aru.stop();   eo_aru.join();

    // 캡처 정지 (wake/push 발생 주체를 먼저 끊는다)
    ir_cap.stop();   ir_cap.join();
    eo_cap.stop();   eo_cap.join();

    // 마지막에 Tx 정지 (GStreamer 파이프라인 보유자)
    ir_tx.stop();    ir_tx.join();
    eo_tx.stop();    eo_tx.join();

    return 0;
}
