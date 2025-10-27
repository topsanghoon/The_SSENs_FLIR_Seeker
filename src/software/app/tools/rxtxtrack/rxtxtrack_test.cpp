// rxtxtrack_test.cpp (AppConfig 주입형 리팩토링판)
// ================================================================
// 빌드 전제:
//  - EO_TxThread / IR_TxThread / IR_TrackThread / Net_RxThread / Meta_TxThread
//    는 이번에 정리한 "주입 기반" 인터페이스를 사용
//  - 공통 로거/타이머/CSV 플래그: app/util/common_log.hpp, time_util.hpp, telemetry.hpp
//  - EventBus 구현: ipc/event_bus_impl.hpp
// ================================================================

#include <atomic>
#include <chrono>
#include <csignal>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>

// ===== 프로젝트 스레드/컴포넌트 =====
#include "threads_includes/IR_TxThread.hpp"
#include "threads_includes/EO_TxThread.hpp"
#include "threads_includes/IR_TrackThread.hpp"
#include "threads_includes/Net_RxThread.hpp"
#include "threads_includes/Meta_TxThread.hpp"

#include "components/includes/IR_Frame.hpp"
#include "components/includes/EO_Frame.hpp"
#include "components/includes/IR_Preprocessor.hpp"   // 실제 전처리
#include "components/includes/IR_Tracker_MOSSE.hpp"  // 실제 트래커
#include "components/includes/CsvLoggerIR.hpp"       // 실제 CSV 로거

#include "ipc/ipc_types.hpp"
#include "ipc/mailbox.hpp"
#include "ipc/wake.hpp"
#include "ipc/event_bus_impl.hpp"    // EventBus 구현

// ✅ 주입형 설정(모든 IP/포트/프레임 파라미터는 여기서)
#include "main_config.hpp"
// ✅ 공통 로거
#include "util/common_log.hpp"

namespace fs = std::filesystem;
using namespace std::chrono_literals;

static std::atomic<bool> g_quit{false};
static void signal_handler(int sig){
    std::cout << "\n[SYS] caught signal " << sig << ", shutting down...\n";
    g_quit.store(true);
}

// ===== [TEST HARNESS] BEGIN =====================================
// 이 블록은 “테스트를 위한 보조 코드”이며, 스레드들(프로덕션)은 수정하지 않음.
namespace flir::test {

// 더미 웨이크 핸들(EO/IR Tx 스레드의 인터페이스 요구 충족용)
struct DummyWakeHandle : public WakeHandle {
    void signal() override {} // no-op
};

// 샘플 파일 수집
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
static std::vector<std::string> collect_jpegs(const std::string& dir) {
    std::vector<std::string> files;
    for (int i=0; i<=9999; ++i) {
        char name[64];
        std::snprintf(name, sizeof(name), "result%03d.jpg", i);
        fs::path p = fs::path(dir) / name;
        if (fs::exists(p)) files.push_back(p.string());
        else if (i > 10) break;
    }
    return files;
}

// IR/EO FrameHandle 구현 (cv::Mat 보존을 위한 테스트 핸들)
struct IR_MatHandle : IRFrameHandle {
    std::shared_ptr<cv::Mat> keep;
    IRFrame16 owned{};
    IR_MatHandle(){ p = &owned; }
    void release() override { keep.reset(); }
};
struct EO_MatHandle : EOFrameHandle {
    std::shared_ptr<cv::Mat> keep;
    FrameBGR8 owned{};
    EO_MatHandle(){ p = &owned; }
    void release() override { keep.reset(); }
    void retain()  override {}
};

// EO 파일 → EO_TxThread 생산자
static void run_eo_producer(const std::vector<std::string>& files,
                            flir::SpscMailbox<std::shared_ptr<flir::EOFrameHandle>>& mb,
                            flir::WakeHandle* wake,  // ★ EO_TxThread 깨우기용
                            int fps,
                            int out_w, int out_h)
{
    const int period_ms = 1000 / std::max(1, fps);
    uint32_t seq = 1;

    for (const auto& file_path : files) {
        if (g_quit.load()) break;
        auto t0 = std::chrono::steady_clock::now();

        cv::Mat m = cv::imread(file_path, cv::IMREAD_COLOR);
        if (m.empty()) { std::cerr << "[EO] load fail: " << file_path << "\n"; continue; }
        if (m.type()!=CV_8UC3) cv::cvtColor(m, m, cv::COLOR_GRAY2BGR);
        if (m.cols != out_w || m.rows != out_h) cv::resize(m, m, cv::Size(out_w, out_h));

        auto h = std::make_shared<flir::test::EO_MatHandle>();
        h->keep = std::make_shared<cv::Mat>(std::move(m));
        h->owned.data   = reinterpret_cast<uint8_t*>(h->keep->data);
        h->owned.width  = h->keep->cols;
        h->owned.height = h->keep->rows;
        h->owned.step   = static_cast<int>(h->keep->step);
        h->seq = seq++;
        h->ts  = (uint64_t)h->seq * (1000000000ULL / fps);

        mb.push(h);
        if (wake) wake->signal();  // ★ EO_TxThread 깨우기 — 핵심!

        int elapsed = (int)std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::steady_clock::now() - t0).count();
        int wait_ms = std::max(1, period_ms - elapsed);
        std::this_thread::sleep_for(std::chrono::milliseconds(wait_ms));
    }
}

} // namespace flir::test
// ===== [TEST HARNESS] END =======================================

int main(int argc, char** argv)
{
    // 인자: [IR_DIR] [EO_DIR] [PC_IP]
    std::string ir_dir = "/home/user/project_dir/The_SSENs_FLIR_Seeker/data/FLIR_Sample";
    std::string eo_dir = "/home/user/project_dir/The_SSENs_FLIR_Seeker/data/EO_Sample";
    std::string pc_ip  = "192.168.2.191";

    if (argc >= 2) ir_dir = argv[1];
    if (argc >= 3) eo_dir = argv[2];
    if (argc >= 4) pc_ip  = argv[3];

    std::signal(SIGINT,  signal_handler);
    std::signal(SIGTERM, signal_handler);

    auto ir_files = flir::test::collect_tiffs(ir_dir);
    auto eo_files = flir::test::collect_jpegs(eo_dir);
    if (ir_files.empty()) { std::cerr << "[ERR] no IR tiffs in " << ir_dir << "\n"; return 1; }
    if (eo_files.empty()) { std::cerr << "[ERR] no EO jpgs  in " << eo_dir << "\n"; return 1; }

    // ========= AppConfig 주입 =========
    auto cfg = std::make_shared<flir::AppConfig>();

    // Net_Rx (클릭 수신)
    cfg->net_rx.port           = 5000;
    cfg->net_rx.buffer_size    = 2048;
    cfg->net_rx.timeout_ms     = 50;
    cfg->net_rx.click_box_size = 40.0f;

    // EO Tx
    cfg->eo_tx.frame.width     = 640;
    cfg->eo_tx.frame.height    = 480;
    cfg->eo_tx.fps             = 15;
    cfg->eo_tx.jpeg_quality    = 30;
    cfg->eo_tx.dst.ip          = pc_ip;
    cfg->eo_tx.dst.port        = 5003;

    // IR Tx
    cfg->ir_tx.frame.width     = 160;
    cfg->ir_tx.frame.height    = 120;
    cfg->ir_tx.fps             = 9;
    cfg->ir_tx.bitDepth        = 16;           // 16bit(raw16)
    cfg->ir_tx.dst.ip          = pc_ip;
    cfg->ir_tx.dst.port        = 5002;

    // Meta Tx
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
    flir::SpscMailbox<std::shared_ptr<flir::EOFrameHandle>> mb_eo(2);

    // EO/IR Tx는 현재 구현에서 WakeHandle을 요구하므로 더미를 전달(IR은 내부에서 자체 깨우기 가능)
    flir::test::DummyWakeHandle dummy_wake;

    flir::IR_TxThread ir_tx("IR_TX", cfg, mb_ir, dummy_wake);
    flir::EO_TxThread eo_tx("EO_TX", cfg, mb_eo, dummy_wake);

    // ★ EO 스레드를 깨울 condvar 핸들 획득
    auto eo_wake_uptr = eo_tx.create_wake_handle();
    flir::WakeHandle* eo_wake = eo_wake_uptr.get();

    // Meta_Tx (META: UDP 5001)
    flir::Meta_TxThread meta_tx(bus, cfg);

    // IR_TrackThread (실제 전처리/트래커/로거 사용)
    flir::SpscMailbox<std::shared_ptr<flir::IRFrameHandle>> mb_ir_trk(2);
    flir::SpscMailbox<flir::UserCmd>                       mb_click_trk(64);

    // 실제 구현체들: IR_Preprocessor(14bit 정규화), MOSSE 트래커, CSV 로거
    flir::IR_Preprocessor preproc(1.0f/16383.0f, 0.0f);
    flir::IR_Tracker_MOSSE tracker;
    flir::CsvLoggerIR      csv_logger("/tmp/rxtxtrack_track.csv");

    flir::IRTrackConfig trk_cfg{};
    trk_cfg.user_req_threshold = 15;

    flir::IR_TrackThread ir_trk(mb_ir_trk, mb_click_trk,
                                tracker, preproc, bus, csv_logger, trk_cfg);

    try {
        // 시작 순서: 수신/메타 → Tx → 트래킹
        net_rx.start();
        meta_tx.start();
        ir_tx.start();
        eo_tx.start();
        ir_trk.start();

        std::cout << "[INFO] listen :5000 (click), META→" << pc_ip << ":5001\n";
        std::cout << "[INFO] IR→" << pc_ip << ":5002 (gray16), EO→" << pc_ip << ":5003 (jpeg)\n";

        // IR 파일 → IR_Tx + IR_Track (TEST HARNESS)
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

        // EO 파일 → EO_Tx (TEST HARNESS)
        std::thread eo_prod([&]{
            flir::test::run_eo_producer(
                eo_files, mb_eo, eo_wake,  // ★ wake 전달
                cfg->eo_tx.fps, cfg->eo_tx.frame.width, cfg->eo_tx.frame.height
            );
        });

        // 메인 루프: 클릭 수신→트래킹 포워딩 (TEST HARNESS)
        while (!g_quit.load()) {
            if (auto cmd = mb_click_rx.exchange(nullptr)) {
                ir_trk.onClickArrived(*cmd);
            }
            std::this_thread::sleep_for(10ms);
        }

        // 종료
        std::cout << "[STOP] stopping threads...\n";
        net_rx.stop();  net_rx.join();
        meta_tx.stop(); meta_tx.join();
        ir_tx.stop();   ir_tx.join();
        eo_tx.stop();   eo_tx.join();
        ir_trk.stop();  ir_trk.join();

        // 테스트 생산자 종료 대기
        ir_prod.join();
        eo_prod.join();

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
        } catch(...) {}
        return 1;
    }
    return 0;
}
