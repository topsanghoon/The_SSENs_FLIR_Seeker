// ================================================================
// 6개 스레드: Net_Rx, Meta_Tx, IR_Tx, EO_Tx, IR_Track, EO_ArUco
// EO는 웹캠 캡처 → EO_Tx로 RTP JPEG 송출 + EO_ArUco에 프레임 전달(콜백)
// ================================================================

#include <atomic>
#include <chrono>
#include <csignal>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>
#include <functional>

#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>

#include "threads_includes/IR_TxThread.hpp"
#include "threads_includes/EO_TxThread.hpp"
#include "threads_includes/IR_TrackThread.hpp"
#include "threads_includes/EO_ArUcoThread.hpp"   // ★ 추가
#include "threads_includes/Net_RxThread.hpp"
#include "threads_includes/Meta_TxThread.hpp"

#include "components/includes/IR_Frame.hpp"
#include "components/includes/EO_Frame.hpp"
#include "components/includes/IR_Preprocessor.hpp"
#include "components/includes/IR_Tracker_MOSSE.hpp"
#include "components/includes/EO_ArucoDetector_OpenCV.hpp" // ★ 추가(오픈CV 디텍터)
#include "components/includes/CsvLoggerIR.hpp"
#include "components/includes/CsvLoggerAru.hpp"            // ★ 추가(아루코 로거)

#include "ipc/ipc_types.hpp"
#include "ipc/mailbox.hpp"
#include "ipc/wake.hpp"
#include "ipc/event_bus_impl.hpp"

#include "main_config.hpp"
#include "util/common_log.hpp"

namespace fs = std::filesystem;
using namespace std::chrono_literals;

static std::atomic<bool> g_quit{false};
static void signal_handler(int sig){
    std::cout << "\n[SYS] caught signal " << sig << ", shutting down...\n";
    g_quit.store(true);
}

namespace flir::test {

struct DummyWakeHandle : public WakeHandle {
    void signal() override {}
};

// 샘플 IR 파일 수집(그대로 유지)
static std::vector<std::string> collect_tiffs(const std::string& dir) {
    std::vector<std::string> files;
    for (int i=0; i<=9999; ++i) {
        char name[64];
        std::snprintf(name, sizeof(name), "frame_%06d.tiff", i);
        fs::path p = fs::path(dir) / name;
        if (fs::exists(p)) files.push_back(p.string());
        else if (i > 10) break;
    }
    return files;
}

// EO 핸들(웹캠 프레임 유지)
struct EO_MatHandle : EOFrameHandle {
    std::shared_ptr<cv::Mat> keep;
    FrameBGR8 owned{};
    EO_MatHandle(){ p = &owned; }
    void release() override { keep.reset(); }
    void retain()  override {}
};

// IR 핸들(파일 프레임 유지)
struct IR_MatHandle : IRFrameHandle {
    std::shared_ptr<cv::Mat> keep;
    IRFrame16 owned{};
    IR_MatHandle(){ p = &owned; }
    void release() override { keep.reset(); }
};

// === EO 웹캠 캡처 → EO_Tx + (콜백으로) EO_ArUco 깨우기 ===
// aruco_push: EO_ArUcoThread::onFrameArrived(h)를 넘겨받아 호출
static void run_eo_camera_producer(int cam_index,
                                   int out_w, int out_h, int fps,
                                   flir::SpscMailbox<std::shared_ptr<flir::EOFrameHandle>>& mb_tx,
                                   std::function<void(std::shared_ptr<flir::EOFrameHandle>)> aruco_push,
                                   flir::WakeHandle* wake_tx)
{
    cv::VideoCapture cap;
    // 선호: V4L2 → 실패 시 CAP_ANY
    if (!cap.open(cam_index, cv::CAP_V4L2)) {
        if (!cap.open(cam_index, cv::CAP_ANY)) {
            std::cerr << "[EO] cannot open camera index " << cam_index << "\n";
            return;
        }
    }

    // 요청 해상도/프레임
    cap.set(cv::CAP_PROP_FRAME_WIDTH,  out_w);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, out_h);
    cap.set(cv::CAP_PROP_FPS,          fps);

    std::cout << "[EO] camera opened: " 
              << (int)cap.get(cv::CAP_PROP_FRAME_WIDTH) << "x"
              << (int)cap.get(cv::CAP_PROP_FRAME_HEIGHT)
              << " @" << (int)cap.get(cv::CAP_PROP_FPS) << "fps\n";

    const int period_ms = 1000 / std::max(1, fps);
    uint32_t seq = 1;

    while (!g_quit.load()) {
        auto t0 = std::chrono::steady_clock::now();
        cv::Mat m;
        if (!cap.read(m) || m.empty()) {
            std::this_thread::sleep_for(5ms);
            continue;
        }
        if (m.cols != out_w || m.rows != out_h) cv::resize(m, m, cv::Size(out_w, out_h));
        if (m.type() != CV_8UC3) cv::cvtColor(m, m, cv::COLOR_GRAY2BGR);

        auto h = std::make_shared<flir::test::EO_MatHandle>();
        h->keep = std::make_shared<cv::Mat>(std::move(m));
        h->owned.data   = reinterpret_cast<uint8_t*>(h->keep->data);
        h->owned.width  = h->keep->cols;
        h->owned.height = h->keep->rows;
        h->owned.step   = static_cast<int>(h->keep->step);
        h->seq = seq++;
        h->ts  = (uint64_t)h->seq * (1000000000ULL / fps);

        // 1) EO_Tx로 전송
        mb_tx.push(h);
        if (wake_tx) wake_tx->signal(); // ★ EO_TxThread 깨우기

        // 2) EO_ArUcoThread에 프레임 투입(= 내부에서 push+notify)
        aruco_push(h); // ★ 핵심

        int elapsed = (int)std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::steady_clock::now() - t0).count();
        int wait_ms = std::max(1, period_ms - elapsed);
        std::this_thread::sleep_for(std::chrono::milliseconds(wait_ms));
    }
}

} // namespace flir::test

int main(int argc, char** argv)
{
    // 인자: [IR_DIR] [PC_IP] [CAM_INDEX]
    std::string ir_dir = "/home/user/project_dir/The_SSENs_FLIR_Seeker/data/FLIR_Sample";
    std::string pc_ip  = "192.168.2.191";
    int cam_index = 0;

    if (argc >= 2) ir_dir = argv[1];
    if (argc >= 3) pc_ip  = argv[2];
    if (argc >= 4) cam_index = std::stoi(argv[3]);

    std::signal(SIGINT,  signal_handler);
    std::signal(SIGTERM, signal_handler);

    auto ir_files = flir::test::collect_tiffs(ir_dir);
    if (ir_files.empty()) { std::cerr << "[ERR] no IR tiffs in " << ir_dir << "\n"; return 1; }

    // ========= AppConfig =========
    auto cfg = std::make_shared<flir::AppConfig>();

    // Net_Rx (클릭 수신)
    cfg->net_rx.port           = 5000;
    cfg->net_rx.buffer_size    = 2048;
    cfg->net_rx.timeout_ms     = 50;
    cfg->net_rx.click_box_size = 40.0f;

    // EO Tx (JPEG → 5003)
    cfg->eo_tx.frame.width     = 640;
    cfg->eo_tx.frame.height    = 480;
    cfg->eo_tx.fps             = 15;
    cfg->eo_tx.jpeg_quality    = 30;
    cfg->eo_tx.dst.ip          = pc_ip;
    cfg->eo_tx.dst.port        = 5003;

    // IR Tx (GRAY16 → 5002)
    cfg->ir_tx.frame.width     = 160;
    cfg->ir_tx.frame.height    = 120;
    cfg->ir_tx.fps             = 9;
    cfg->ir_tx.bitDepth        = 16;
    cfg->ir_tx.dst.ip          = pc_ip;
    cfg->ir_tx.dst.port        = 5002;

    // Meta Tx (→ 5001)
    cfg->meta_tx.hb_period_ms  = 1000;
    cfg->meta_tx.sndbuf_bytes  = 256*1024;
    cfg->meta_tx.local_port    = 0;
    cfg->meta_tx.dst.ip        = pc_ip;
    cfg->meta_tx.dst.port      = 5001;

    // ========= 스레드/IPC =========
    flir::EventBus bus;

    // 클릭 수신(UDP:5000)
    flir::SpscMailbox<flir::UserCmd> mb_click_rx(64);
    flir::Net_RxThread net_rx("Net_Rx", cfg, mb_click_rx);

    // IR/EO Tx
    flir::SpscMailbox<std::shared_ptr<flir::IRFrameHandle>> mb_ir(2);
    flir::SpscMailbox<std::shared_ptr<flir::EOFrameHandle>> mb_eo_tx(2);   // EO_Tx 용

    // EO 아루코용 메일박스 (EO_ArUcoThread 내부 push로 사용)
    flir::SpscMailbox<std::shared_ptr<flir::EOFrameHandle>> mb_eo_aru(2);

    // 더미 웨이크 (IR/EO Tx가 요구)
    flir::test::DummyWakeHandle dummy_wake;

    flir::IR_TxThread ir_tx("IR_TX", cfg, mb_ir, dummy_wake);
    flir::EO_TxThread eo_tx("EO_TX", cfg, mb_eo_tx, dummy_wake);

    // EO_Tx 깨우기 핸들
    auto eo_wake_uptr = eo_tx.create_wake_handle();
    flir::WakeHandle* eo_wake = eo_wake_uptr.get();

    // Meta Tx
    flir::Meta_TxThread meta_tx(bus, cfg);

    // IR Track
    flir::SpscMailbox<std::shared_ptr<flir::IRFrameHandle>> mb_ir_trk(2);
    flir::SpscMailbox<flir::UserCmd>                       mb_click_trk(64);

    flir::IR_Preprocessor preproc(1.0f/16383.0f, 0.0f);
    flir::IR_Tracker_MOSSE tracker;
    flir::CsvLoggerIR      csv_logger("/tmp/rxtxtrack_track.csv");

    flir::IRTrackConfig trk_cfg{};
    trk_cfg.user_req_threshold = 15;

    flir::IR_TrackThread ir_trk(mb_ir_trk, mb_click_trk, tracker, preproc, bus, trk_cfg);

    // EO ArUco (OpenCV detector 사용)
    flir::CsvLoggerAru         csv_logger_aru("/tmp/rxtxtrack_aruco.csv");
    flir::EO_ArucoDetector_OpenCV aruco_detector(cv::aruco::DICT_4X4_50); // 필요시 딕셔너리 맞추기
    // 간단 전처리: BGR->GRAY 변환만 하는 프리프로세서
    struct SimpleArucoPreproc : flir::IArucoPreprocessor {
        void run(const flir::EOFrameHandle& in, cv::Mat& pf_gray8) override {
            const flir::FrameBGR8& f = *in.p;
            cv::Mat src(f.height, f.width, CV_8UC3, (void*)f.data, f.step);
            cv::cvtColor(src, pf_gray8, cv::COLOR_BGR2GRAY);
        }
    } aruco_preproc;

    flir::EO_ArUcoThread eo_aru(mb_eo_aru, aruco_preproc, aruco_detector, bus);

    try {
        // 시작
        net_rx.start();
        meta_tx.start();
        ir_tx.start();
        eo_tx.start();
        ir_trk.start();
        eo_aru.start();

        std::cout << "[INFO] listen :5000 (click), META→" << pc_ip << ":5001\n";
        std::cout << "[INFO] IR→" << pc_ip << ":5002 (gray16), EO→" << pc_ip << ":5003 (jpeg)\n";
        std::cout << "[INFO] EO webcam index=" << cam_index << "\n";

        // IR 파일 → IR_Tx + IR_Track
        std::thread ir_prod([&]{
            const int fps = cfg->ir_tx.fps;
            const int period_ms = 1000 / std::max(1, fps);
            uint32_t seq = 1;

            for (const auto& file_path : ir_files) {
                if (g_quit.load()) break;
                auto t0 = std::chrono::steady_clock::now();

                cv::Mat m = cv::imread(file_path, cv::IMREAD_UNCHANGED);
                if (m.empty()) { std::cerr << "[IR] load fail: " << file_path << "\n"; continue; }
                if (m.type() != CV_16UC1) {
                    if (m.channels()!=1) cv::cvtColor(m, m, cv::COLOR_BGR2GRAY);
                    m.convertTo(m, CV_16U, 65535.0/255.0);
                }
                if (m.cols!=cfg->ir_tx.frame.width || m.rows!=cfg->ir_tx.frame.height) {
                    cv::resize(m, m, cv::Size(cfg->ir_tx.frame.width, cfg->ir_tx.frame.height));
                }

                auto h = std::make_shared<flir::test::IR_MatHandle>();
                h->keep = std::make_shared<cv::Mat>(std::move(m));
                h->owned.data   = reinterpret_cast<uint16_t*>(h->keep->data);
                h->owned.width  = h->keep->cols;
                h->owned.height = h->keep->rows;
                h->owned.step   = static_cast<int>(h->keep->step);
                h->seq = seq++;
                h->ts  = (uint64_t)h->seq * (1000000000ULL / fps);

                // 1) IR Tx
                mb_ir.push(h);
                // 2) IR Track
                ir_trk.onFrameArrived(h);

                int elapsed = (int)std::chrono::duration_cast<std::chrono::milliseconds>(
                                std::chrono::steady_clock::now() - t0).count();
                int wait_ms = std::max(1, period_ms - elapsed);
                std::this_thread::sleep_for(std::chrono::milliseconds(wait_ms));
            }
        });

        // EO 웹캠 → EO_Tx + EO_ArUco
        std::thread eo_cam_prod([&]{
            flir::test::run_eo_camera_producer(
                cam_index,
                cfg->eo_tx.frame.width, cfg->eo_tx.frame.height, cfg->eo_tx.fps,
                mb_eo_tx,
                [&](std::shared_ptr<flir::EOFrameHandle> h){ eo_aru.onFrameArrived(std::move(h)); }, // ★ 콜백
                eo_wake
            );
        });

        // 메인 루프: 클릭 수신→트래킹 전달
        while (!g_quit.load()) {
            if (auto cmd = mb_click_rx.exchange(nullptr)) {
                ir_trk.onClickArrived(*cmd);
            }
            std::this_thread::sleep_for(10ms);
        }

        std::cout << "[STOP] stopping threads...\n";
        net_rx.stop();  net_rx.join();
        meta_tx.stop(); meta_tx.join();
        ir_tx.stop();   ir_tx.join();
        eo_tx.stop();   eo_tx.join();
        ir_trk.stop();  ir_trk.join();
        eo_aru.stop();  eo_aru.join();

        ir_prod.join();
        eo_cam_prod.join();

        std::cout << "[DONE]\n";
    }
    catch(const std::exception& e){
        std::cerr << "[ERR] exception: " << e.what() << "\n";
        try {
            net_rx.stop();  net_rx.join();
            meta_tx.stop(); meta_tx.join();
            ir_tx.stop();   ir_tx.join();
            eo_tx.stop();   eo_tx.join();
            ir_trk.stop();  ir_trk.join();
            eo_aru.stop();  eo_aru.join();
        } catch(...) {}
        return 1;
    }
    return 0;
}
