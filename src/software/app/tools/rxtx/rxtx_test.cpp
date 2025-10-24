#include <iostream>
#include <string>
#include <vector>
#include <chrono>
#include <thread>
#include <filesystem>
#include <atomic>
#include <csignal>
#include <iomanip>
#include <memory>
#include <algorithm>

// Linux UDP
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>

// OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>

// ==== 프로젝트 스레드/IPC ====
#include "threads_includes/Net_RxThread.hpp"
#include "threads_includes/IR_TxThread.hpp"
#include "threads_includes/EO_TxThread.hpp"

#include "components/includes/IR_Frame.hpp"
#include "components/includes/EO_Frame.hpp"

#include "ipc/ipc_types.hpp"
#include "ipc/mailbox.hpp"
#include "ipc/wake.hpp"  // WakeHandle 선언

namespace fs = std::filesystem;
using namespace std::chrono_literals;

static std::atomic<bool> g_quit{false};

static void signal_handler(int sig){
    std::cout << "\n[SYS] caught signal " << sig << ", shutting down...\n";
    g_quit.store(true);
}

// ---------------- UdpSender (메타 텍스트 송신) ----------------
class UdpSender {
public:
    UdpSender(const std::string& ip, uint16_t port) {
        sock_ = ::socket(AF_INET, SOCK_DGRAM, 0);
        if (sock_ < 0) throw std::runtime_error("socket() failed");

        std::memset(&addr_, 0, sizeof(addr_));
        addr_.sin_family = AF_INET;
        addr_.sin_port   = htons(port);
        if (::inet_pton(AF_INET, ip.c_str(), &addr_.sin_addr) != 1) {
            ::close(sock_);
            throw std::runtime_error("inet_pton() failed for " + ip);
        }
    }
    ~UdpSender(){ if (sock_>=0) ::close(sock_); }

    void send(const std::string& s){
        ::sendto(sock_, s.data(), (int)s.size(), 0, (sockaddr*)&addr_, sizeof(addr_));
    }
private:
    int sock_{-1};
    sockaddr_in addr_{};
};

// ---------------- 샘플 파일 수집 ----------------
static std::vector<std::string> collect_tiffs(const std::string& dir) {
    std::vector<std::string> files;
    for (int i=0; i<=9999; i++) {
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
    for (int i=0; i<=9999; i++) {
        char name[64];
        std::snprintf(name, sizeof(name), "result%03d.jpg", i);
        fs::path p = fs::path(dir) / name;
        if (fs::exists(p)) files.push_back(p.string());
        else if (i > 10) break;
    }
    return files;
}

// ---------------- IR/EO 프레임 핸들 ----------------
struct IR_MatHandle : flir::IRFrameHandle {
    std::shared_ptr<cv::Mat> keep;
    flir::IRFrame16 owned{};
    IR_MatHandle(){ p = &owned; }
    void release() override { keep.reset(); }
};
struct EO_MatHandle : flir::EOFrameHandle {
    std::shared_ptr<cv::Mat> keep;
    flir::FrameBGR8 owned{};
    EO_MatHandle(){ p = &owned; }
    void release() override { keep.reset(); }
    void retain()  override { /* no-op */ }
};

// ---------------- 생산자 루프 (IR) ----------------
static void run_ir_producer(const std::vector<std::string>& files,
                            flir::SpscMailbox<std::shared_ptr<flir::IRFrameHandle>>& mb,
                            flir::WakeHandle* wake,
                            int fps)
{
    const int period_ms = 1000 / std::max(1, fps);
    uint32_t seq = 1;

    for (const auto& file_path : files) {
        if (g_quit.load()) break;

        auto t0 = std::chrono::steady_clock::now();

        cv::Mat m = cv::imread(file_path, cv::IMREAD_UNCHANGED);
        if (m.empty()) { std::cerr << "[IR] load fail: " << file_path << "\n"; continue; }
        if (m.type() != CV_16UC1) {
            if (m.channels()!=1) cv::cvtColor(m, m, cv::COLOR_BGR2GRAY);
            m.convertTo(m, CV_16U, 65535.0/255.0);
        }
        if (m.cols!=160 || m.rows!=120) cv::resize(m, m, cv::Size(160,120));

        auto h = std::make_shared<IR_MatHandle>();
        h->keep = std::make_shared<cv::Mat>(std::move(m));
        h->owned.data   = reinterpret_cast<uint16_t*>(h->keep->data);
        h->owned.width  = h->keep->cols;
        h->owned.height = h->keep->rows;
        h->owned.step   = static_cast<int>(h->keep->step);
        h->seq = seq++;
        h->ts  = (uint64_t)h->seq * (1000000000ULL / fps);

        mb.push(h);
        if (wake) wake->signal();

        // pace
        int elapsed = (int)std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::steady_clock::now() - t0).count();
        int wait_ms = std::max(1, period_ms - elapsed);
        std::this_thread::sleep_for(std::chrono::milliseconds(wait_ms));
    }
}

// ---------------- 생산자 루프 (EO) ----------------
static void run_eo_producer(const std::vector<std::string>& files,
                            flir::SpscMailbox<std::shared_ptr<flir::EOFrameHandle>>& mb,
                            flir::WakeHandle* wake,
                            int fps)
{
    const int period_ms = 1000 / std::max(1, fps);
    uint32_t seq = 1;

    for (const auto& file_path : files) {
        if (g_quit.load()) break;

        auto t0 = std::chrono::steady_clock::now();

        cv::Mat m = cv::imread(file_path, cv::IMREAD_COLOR);
        if (m.empty()) { std::cerr << "[EO] load fail: " << file_path << "\n"; continue; }
        if (m.type()!=CV_8UC3) cv::cvtColor(m, m, cv::COLOR_GRAY2BGR);

        auto h = std::make_shared<EO_MatHandle>();
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
    // ------- 인자: [IR_DIR] [EO_DIR] [PC_IP] -------
    std::string ir_dir = "/home/user/project_dir/The_SSENs_FLIR_Seeker/data/FLIR_Sample";
    std::string eo_dir = "/home/user/project_dir/The_SSENs_FLIR_Seeker/data/EO_Sample";
    std::string pc_ip  = "192.168.2.191"; // WPF PC IP

    if (argc >= 2) ir_dir = argv[1];
    if (argc >= 3) eo_dir = argv[2];
    if (argc >= 4) pc_ip  = argv[3];

    std::signal(SIGINT,  signal_handler);
    std::signal(SIGTERM, signal_handler);

    // ------- 파일 로드 -------
    auto ir_files = collect_tiffs(ir_dir);
    auto eo_files = collect_jpegs(eo_dir);

    if (ir_files.empty()) { std::cerr << "[ERR] no IR tiffs in " << ir_dir << "\n"; return 1; }
    if (eo_files.empty()) { std::cerr << "[ERR] no EO jpgs  in " << eo_dir << "\n"; return 1; }

    // ------- 메타 텍스트 송신기 (→ PC:5001) -------
    UdpSender meta(pc_ip, 5001);
    meta.send("[META] combo test starting");

    // ------- 명령 수신(← PC:5000) 준비 -------
    flir::SpscMailbox<flir::UserCmd> mb_click(64);
    flir::NetRxConfig rx_cfg;
    rx_cfg.port           = 5000;      // WPF가 보내는 포트
    rx_cfg.buffer_size    = 2048;
    rx_cfg.timeout_ms     = 50;
    rx_cfg.click_box_size = 40.0f;     // 클릭 점에서 생성할 초기 박스 크기
    flir::Net_RxThread net_rx(mb_click, rx_cfg);

    // ------- IR Tx 스레드 -------
    flir::SpscMailbox<std::shared_ptr<flir::IRFrameHandle>> mb_ir(2);
    flir::IR_TxThread ir_tx("IR_TX", mb_ir);
    auto ir_wake_uptr = ir_tx.create_wake_handle(); // unique_ptr<WakeHandle>
    flir::WakeHandle* ir_wake = ir_wake_uptr.get(); // raw pointer로 사용

    // ------- EO Tx 스레드 -------
    flir::SpscMailbox<std::shared_ptr<flir::EOFrameHandle>> mb_eo(2);
    flir::EO_TxThread eo_tx("EO_TX", mb_eo);
    auto eo_wake_uptr = eo_tx.create_wake_handle(); // unique_ptr<WakeHandle>
    flir::WakeHandle* eo_wake = eo_wake_uptr.get();

    try {
        // 스레드 시작
        net_rx.start();
        ir_tx.start();
        eo_tx.start();

        std::cout << "[INFO] listening cmd on :5000, sending meta to " << pc_ip << ":5001\n";
        std::cout << "[INFO] IR stream → PC:5002, EO stream → PC:5003\n";

        // 생산자(파일→메일박스) 루프는 std::thread로 병렬 실행
        std::thread ir_prod([&]{ run_ir_producer(ir_files, mb_ir, ir_wake, 9);   });
        std::thread eo_prod([&]{ run_eo_producer(eo_files, mb_eo, eo_wake, 30); });

        // 메타 heartbeat
        auto t0 = std::chrono::steady_clock::now();
        uint64_t hb = 0;

        while (!g_quit.load()) {
            // 명령 수신 처리
            if (auto cmd = mb_click.exchange(nullptr)) {
                std::ostringstream oss;
                oss << "[META] RX cmd seq=" << cmd->seq
                    << " type=" << int(cmd->type)
                    << " box=(" << std::fixed << std::setprecision(1)
                    << cmd->box.x << "," << cmd->box.y << ","
                    << cmd->box.width << "," << cmd->box.height << ")";
                auto s = oss.str();
                std::cout << s << "\n";
                meta.send(s);
            }

            // 1초마다 heartbeat 메타
            auto now = std::chrono::steady_clock::now();
            if (std::chrono::duration_cast<std::chrono::seconds>(now - t0).count() >= 1) {
                t0 = now;
                std::ostringstream oss;
                oss << "[META] hb=" << ++hb;
                meta.send(oss.str());
            }

            std::this_thread::sleep_for(10ms);
        }

        // 종료
        std::cout << "[STOP] stopping threads...\n";
        net_rx.stop();  net_rx.join();
        ir_tx.stop();   ir_tx.join();
        eo_tx.stop();   eo_tx.join();
        ir_prod.join();
        eo_prod.join();

        meta.send("[META] combo test stopped");
        std::cout << "[DONE]\n";
    }
    catch(const std::exception& e){
        std::cerr << "[ERR] exception: " << e.what() << "\n";
        try {
            net_rx.stop();  net_rx.join();
            ir_tx.stop();   ir_tx.join();
            eo_tx.stop();   eo_tx.join();
        } catch(...) {}
        return 1;
    }

    return 0;
}
