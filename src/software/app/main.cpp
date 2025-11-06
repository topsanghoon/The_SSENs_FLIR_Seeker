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

// === 종료 플래그 & 시그널 핸들러 ===
// - 메인 루프에서 폴링하여 전체 종료 시퀀스를 트리거.
// - 최소한의 책임이므로 main에 남아도 OK.
static std::atomic<bool> g_quit{false};
static void sig_handler(int s){ std::cout << "\n[SIG] " << s << " → quit\n"; g_quit.store(true); }

// 더미 웨이크 (인터페이스 충족용)
// - 테스트/데모용으로만 사용.
// - 실제 운용 시 EO_Tx가 자체 웨이크를 생성하므로 이 더미는 제거 권장.
// [MOVE-OUT] 테스트 전용 더미 타입 → tests/ 또는 demo/ 유닛으로 분리.
struct DummyWakeTx  : WakeHandle { void signal() override {} };

int main(int, char**) {
    // === 프로세스 레벨 초기화 ===
    std::signal(SIGINT,  sig_handler);
    std::signal(SIGTERM, sig_handler);

    // === 전역 AppConfig 준비 ===
    auto app = std::make_shared<AppConfig>();

    // ====== Config ======
    // 초기 유도 단계 지정
    app->guidance.default_phase = GuidancePhase::Midcourse;
    GuidanceState::phase().store(app->guidance.default_phase); // 전역 상태 싱크

    // EO 송출 파라미터
    app->eo_tx.frame = {320,240};
    app->eo_tx.fps = 15;
    app->eo_tx.jpeg_quality = 30;
    app->eo_tx.dst = {"192.168.0.18", 5003};

    // IR 송출 파라미터 (Lepton 2.5 기준)
    app->ir_tx.frame = {80,60};
    app->ir_tx.fps   = 9;
    app->ir_tx.bitDepth = 16;
    app->ir_tx.dst = {"192.168.0.18", 5002};

    // 메타/네트 설정
    app->meta_tx.dst        = {"192.168.0.18", 5001};
    app->meta_tx.local_port = 0;              // 0=ephemeral
    app->meta_tx.hb_period_ms = 1000;         // Heartbeat 주기
    app->net_rx.port          = 5000;
    app->net_rx.timeout_ms    = 50;
    app->net_rx.buffer_size   = 2048;
    app->net_rx.click_box_size= 40.0f;

    // 유도 로직 기본 파라미터
    app->guidance.terminal_marker_id = 3;
    app->guidance.min_bbox_w = 80;
    app->guidance.min_bbox_h = 80;
    app->guidance.min_big_frames = 5;
    app->guidance.hold_big_ms = 300;
    app->guidance.min_bbox_frac = 0.0f;


    // ====== IPC (이벤트버스/메일박스) ======
    // - EventBus: 스레드 간 이벤트 브로커
    // - Mailbox(SPSC): 높은 처리량/낮은 지연의 단방향 큐
    // 메인에서 토폴로지만 간략히 정의하는 건 OK.
    // [MOVE-OUT] 단, 큐 크기/토폴로지 설정은 config로 빼면 재사용성↑
    EventBus bus;

    // IR 프레임 소비자별 큐 (Tx/Track 분리)
    SpscMailbox<std::shared_ptr<IRFrameHandle>> mb_ir_tx(8);
    SpscMailbox<std::shared_ptr<IRFrameHandle>> mb_ir_trk(8);

    // EO 프레임 소비자별 큐 (Tx/ArUco 분리)
    SpscMailbox<std::shared_ptr<EOFrameHandle>> mb_eo_tx(8);
    SpscMailbox<std::shared_ptr<EOFrameHandle>> mb_eo_aru(8);

    // 사용자 입력/자폭 커맨드 큐
    SpscMailbox<UserCmd>         mb_click(64);
    SpscMailbox<SelfDestructCmd> mb_sd(4);

    // ====== Threads 구성 ======
    // --- EO 경로 ---
    DummyWakeTx dummy_wake_tx; // [MOVE-OUT] 테스트/데모 코드
    EO_TxThread eo_tx("EO_Tx", app, mb_eo_tx, dummy_wake_tx);
    auto eo_wake_for_cap = eo_tx.create_wake_handle(); // EO 캡처가 프레임을 밀어넣을 때 Tx를 깨우기 위함
    EO_CaptureThread eo_cap("EO_Cap", mb_eo_tx, mb_eo_aru, std::move(eo_wake_for_cap), app);
    // [NOTE] EO는 Tx가 웨이크 소유. 캡처→Tx/ArUco로 fan-out.

    // --- IR 경로 ---
    // 1) IR Tx: (mailbox + GstConfig)
    IR_TxThread::GstConfig ir_gst{};
    ir_gst.width  = app->ir_tx.frame.width;   // 80
    ir_gst.height = app->ir_tx.frame.height;  // 60
    ir_gst.fps    = app->ir_tx.fps;           // 9
    ir_gst.pc_ip  = app->ir_tx.dst.ip;        // "192.168.0.18"
    ir_gst.port   = app->ir_tx.dst.port;      // 5002

    
    IR_TxThread ir_tx("IR_Tx", mb_ir_tx, ir_gst);

    // 2) IR Capture → (내부 라우팅) TX/Track
    auto ir_wake_for_cap = ir_tx.create_wake_handle();

    IRCaptureConfig ir_cap_cfg{};
    ir_cap_cfg.spi_device      = "/dev/spidev1.0";  // ★ main에서 주입
    ir_cap_cfg.spi_speed       = 2'000'000;        // ★ main에서 주입 (안정성 필요시 16MHz 권장)
    ir_cap_cfg.spi_delay_usecs = 0;                 // ★ main에서 주입
    ir_cap_cfg.fps             = app->ir_tx.fps;    // ★ main에서 주입

    // 캡처 스레드의 출력 메일박스를 "직접" TX 메일박스로 연결
    IR_CaptureThread ir_cap("IR_Cap", mb_ir_tx, mb_ir_trk, std::move(ir_wake_for_cap), ir_cap_cfg);






    // === 분석 스레드 ===
    // IR 전처리/트래커 선택
    IR_Preprocessor  ir_pre(1.0f/16383.0f, 0.0f); // 14-bit → [0,1] 정규화
    IR_Tracker_MOSSE ir_mosse;                    // 경량/저지연 트래커
    IRTrackConfig    trk_cfg{}; trk_cfg.user_req_threshold = 15;
    IR_TrackThread   ir_track(mb_ir_trk, mb_click, ir_mosse, ir_pre, bus, trk_cfg);
    // [MOVE-OUT] 전처리 스케일/오프셋, 트래커 종류/파라미터는 config/플러그인 선택으로

    // EO ArUco 설정
    EO_ArucoDetector_OpenCV aruco(cv::aruco::DICT_4X4_50);
    // 간단한 그레이 변환 전처리
    // [MOVE-OUT] 임시 구현체 SimpleArucoPreproc → components/preproc/ 로 파일 분리
    struct SimpleArucoPreproc : IArucoPreprocessor {
        void run(const EOFrameHandle& in, cv::Mat& gray) override {
            const FrameBGR8& f = *in.p;
            cv::Mat src(f.height, f.width, CV_8UC3, (void*)f.data, f.step);
            cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);
        }
    } arupre;
    EO_ArUcoThread eo_aru(mb_eo_aru, arupre, aruco, bus);
    // [MOVE-OUT] 사용 사전(dict)/파라미터도 설정화

    // === 제어(퓨전/컨트롤/액추에이터) ===
    ControlThread::Config ccfg;
    ccfg.period_ms = 20;                 // 제어 루프 주기
    ccfg.guidance  = app->guidance;      // 유도 파라미터 전달
    ccfg.eo_w      = app->eo_tx.frame.width;
    ccfg.eo_h      = app->eo_tx.frame.height;

    TargetFusion  fusion;                // IR/EO 융합
    ControllerLR  controller(app->eo_tx.frame.width, 4.0f); // 좌/우 제어기 (게인 등)
    // [MOVE-OUT] 컨트롤러 게인/방정식은 설정/튜닝 파일로
    UART_ActuatorPort act("/dev/ttyUL0", B115200);
    // [MOVE-OUT] UART 디바이스/보율은 설정으로
    ControlThread control(bus, mb_sd, fusion, controller, act, ccfg);

    // === 메타/네트 ===
    Meta_TxThread meta_tx(bus, app);                // 상태/이벤트 메타 전송
    Net_RxThread  net_rx("Net_Rx", app, mb_click);  // 지상국 입력 수신
    // [MOVE-OUT] 메시지 포맷/주기/리트라이 등은 설정으로


    // EO
    eo_cap.set_aru_sink([&](std::shared_ptr<flir::EOFrameHandle> h){
        eo_aru.onFrameArrived(std::move(h));
    });

    // IR
    ir_cap.set_track_sink([&](std::shared_ptr<flir::IRFrameHandle> h){
        ir_track.onFrameArrived(std::move(h));
    });


    // ====== Start ======
    // [NOTE] main은 "조립 및 시작/정지"만 담당 → 적절
    ir_cap.start();              // IR 캡처
    eo_cap.start();              // EO 캡처

    ir_tx.start();               // IR 송출
    eo_tx.start();               // EO 송출

    ir_track.start();
    eo_aru.start();

    meta_tx.start();
    net_rx.start();

    control.start();

    std::cout << "[INFO] Midcourse start. Ctrl+C to exit.\n";
    while (!g_quit.load()) std::this_thread::sleep_for(50ms);

    // ====== Stop (역순 정지) ======
    // [MOVE-OUT] 정지 순서를 ThreadManager로 캡슐화하여 start_all()/stop_all() 제공 시 main 간결화
    control.stop();  control.join();
    net_rx.stop();   net_rx.join();
    meta_tx.stop();  meta_tx.join();
    ir_track.stop(); ir_track.join();
    eo_aru.stop();   eo_aru.join();

    ir_tx.stop();    ir_tx.join();
    eo_tx.stop();    eo_tx.join();

    ir_cap.stop();   ir_cap.join();
    eo_cap.stop();   eo_cap.join();

    // splitter 종료
    // splitter_run.store(false);
    // if (ir_splitter.joinable()) ir_splitter.join();

    return 0;
}
