// rxtxtrack_test.cpp
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

namespace fs = std::filesystem;
using namespace std::chrono_literals;

static std::atomic<bool> g_quit{false};
static void signal_handler(int sig){
    std::cout << "\n[SYS] caught signal " << sig << ", shutting down...\n";
    g_quit.store(true);
}

// ------- 샘플 파일 수집 -------
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

// ------- IR/EO FrameHandle 구현 (스마트 포인터 보존) -------
namespace flir {
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
    void retain()  override { /* no-op */ }
};
} // namespace flir

// ------- EO 파일 → EO_TxThread 생산자 -------
static void run_eo_producer(const std::vector<std::string>& files,
                            flir::SpscMailbox<std::shared_ptr<flir::EOFrameHandle>>& mb,
                            flir::WakeHandle* wake,
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

        auto h = std::make_shared<flir::EO_MatHandle>();
        h->keep = std::make_shared<cv::Mat>(std::move(m));
        h->owned.data   = reinterpret_cast<uint8_t*>(h->keep->data);
        h->owned.width  = h->keep->cols;
        h->owned.height = h->keep->rows;
        h->owned.step   = static_cast<int>(h->keep->step);
        h->seq = seq++;
        h->ts  = (uint64_t)h->seq * (1000000000ULL / fps);

        mb.push(h);
        if (wake) wake->signal();

        int elapsed = (int)std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::steady_clock::now() - t0).count();
        int wait_ms = std::max(1, period_ms - elapsed);
        std::this_thread::sleep_for(std::chrono::milliseconds(wait_ms));
    }
}

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

    auto ir_files = collect_tiffs(ir_dir);
    auto eo_files = collect_jpegs(eo_dir);
    if (ir_files.empty()) { std::cerr << "[ERR] no IR tiffs in " << ir_dir << "\n"; return 1; }
    if (eo_files.empty()) { std::cerr << "[ERR] no EO jpgs  in " << eo_dir << "\n"; return 1; }

    // ========= 스레드/IPC =========
    // 클릭 수신(UDP:5000)
    flir::SpscMailbox<flir::UserCmd> mb_click_rx(64);
    flir::NetRxConfig rx_cfg;
    rx_cfg.port           = 5000;
    rx_cfg.buffer_size    = 2048;
    rx_cfg.timeout_ms     = 50;
    rx_cfg.click_box_size = 40.0f;
    flir::Net_RxThread net_rx(mb_click_rx, rx_cfg);

    // IR/EO Tx
    flir::SpscMailbox<std::shared_ptr<flir::IRFrameHandle>> mb_ir(2);
    flir::SpscMailbox<std::shared_ptr<flir::EOFrameHandle>> mb_eo(2);

    flir::IR_TxThread::GstConfig ir_gst{};
    ir_gst.width  = 160;
    ir_gst.height = 120;
    ir_gst.fps    = 9;
    ir_gst.pc_ip  = pc_ip;
    ir_gst.port   = 5002;
    flir::IR_TxThread ir_tx("IR_TX", mb_ir, ir_gst);
    auto ir_wake_uptr = ir_tx.create_wake_handle();
    flir::WakeHandle* ir_wake = ir_wake_uptr.get();

    flir::EO_TxThread::GstConfig eo_gst{};
    eo_gst.width  = 640;
    eo_gst.height = 480;
    eo_gst.fps    = 30;
    eo_gst.pc_ip  = pc_ip;
    eo_gst.port   = 5003;
    flir::EO_TxThread eo_tx("EO_TX", mb_eo, eo_gst);
    auto eo_wake_uptr = eo_tx.create_wake_handle();
    flir::WakeHandle* eo_wake = eo_wake_uptr.get();

    // EventBus & Meta_Tx (META: UDP 5001)
    flir::EventBus bus;
    flir::MetaTxConfig meta_cfg{};
    meta_cfg.hb_period_ms = 1000;
    meta_cfg.sndbuf_bytes = 256*1024;
    meta_cfg.local_port   = 0;
    meta_cfg.remote_port  = 5001;
    std::snprintf(meta_cfg.remote_ip, sizeof(meta_cfg.remote_ip), "%s", pc_ip.c_str());
    flir::MetaFds meta_fds{};
    flir::Meta_TxThread meta_tx(bus, meta_cfg, meta_fds);

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

        // IR 파일 → IR_Tx + IR_Track
        std::thread ir_prod([&]{
            const int fps = 9;
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
                if (m.cols!=ir_gst.width || m.rows!=ir_gst.height) cv::resize(m, m, cv::Size(ir_gst.width, ir_gst.height));

                auto h = std::make_shared<flir::IR_MatHandle>();
                h->keep = std::make_shared<cv::Mat>(std::move(m));
                h->owned.data   = reinterpret_cast<uint16_t*>(h->keep->data);
                h->owned.width  = h->keep->cols;
                h->owned.height = h->keep->rows;
                h->owned.step   = static_cast<int>(h->keep->step);
                h->seq = seq++;
                h->ts  = (uint64_t)h->seq * (1000000000ULL / fps);

                // 1) IR Tx
                mb_ir.push(h);
                if (ir_wake) ir_wake->signal();

                // 2) IR Track
                ir_trk.onFrameArrived(h);

                int elapsed = (int)std::chrono::duration_cast<std::chrono::milliseconds>(
                                std::chrono::steady_clock::now() - t0).count();
                int wait_ms = std::max(1, period_ms - elapsed);
                std::this_thread::sleep_for(std::chrono::milliseconds(wait_ms));
            }
        });

        // EO 파일 → EO_Tx (640x480로 맞춤)
        std::thread eo_prod([&]{ run_eo_producer(eo_files, mb_eo, eo_wake, eo_gst.fps, eo_gst.width, eo_gst.height); });

        // 메인 루프: 클릭 수신→트래킹으로 포워딩 (Net_RxThread→mb_click_rx)
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
