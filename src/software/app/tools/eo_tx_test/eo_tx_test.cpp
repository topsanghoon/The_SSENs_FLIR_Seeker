#include <iostream>
#include <string>
#include <vector>
#include <chrono>
#include <thread>
#include <filesystem>
#include <stdexcept>
#include <atomic>
#include <csignal>

#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>

// === 테스트 대상 스레드 및 관련 타입들 ===
#include "threads_includes/EO_TxThread.hpp"
#include "components/includes/EO_Frame.hpp"
#include "ipc/ipc_types.hpp"
#include "ipc/mailbox.hpp"
#include "ipc/wake.hpp"

namespace fs = std::filesystem;
using namespace std::chrono_literals;

static std::atomic<bool> g_quit{false};

void signal_handler(int signum) {
    std::cout << "\nCaught signal " << signum << ", shutting down." << std::endl;
    g_quit.store(true);
}

// === EO 프레임 핸들 구현 ===
// - EO_TxThread는 EOFrameHandle의 포인터만 소비.
// - 실제 버퍼 소유권은 MatHandle의 shared_ptr<cv::Mat>이 보유하고, release()에서 해제.
struct MatHandle : flir::EOFrameHandle {
    std::shared_ptr<cv::Mat> keep;
    flir::FrameBGR8 owned{};     // 프레임 메타(포인터/폭/높이/스텝)를 내부에서 보유
    MatHandle() { p = &owned; }  // base 포인터(p)가 내가 가진 owned를 가리키도록 연결
    
    // 이 함수는 EO_TxThread의 GStreamer 콜백에서 호출되어 cv::Mat 메모리를 해제합니다.
    void release() override { keep.reset(); } 
    void retain() override { } // no-op for shared_ptr
};

// === 샘플 JPEG 파일 수집 ===
static std::vector<std::string> collect_jpegs(const std::string& dir) {
    std::vector<std::string> files;
    for (int i=0; i<=9999; i++) {
        char name[64];
        std::snprintf(name, sizeof(name), "result%03d.jpg", i);
        fs::path p = fs::path(dir) / name;
        if (fs::exists(p)) {
            files.push_back(p.string());
        } else if (i > 10) { // 너무 많이 찾지 않도록 조기 종료 조건 추가
            break;
        }
    }
    return files;
}

int main(int argc, char** argv) {
    // Ctrl+C 종료 처리
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    // ★ 입력 프레임 소스(샘플 데이터 디렉토리)
    std::string dir = "/home/user/project_dir/The_SSENs_FLIR_Seeker/data/EO_Sample";
    if (argc >= 2) dir = argv[1];

    auto files = collect_jpegs(dir);
    if (files.empty()) {
        std::cerr << "[ERR] No JPEG files found in " << dir << "\n";
        return 1;
    }
    std::cout << "[INFO] Found " << files.size() << " frames to stream.\n";

    // ============================
    //  EO_TxThread 파이프라인 와이어링
    // ============================

    // [프레임 메일박스] (생산자: main 루프 / 소비자: EO_TxThread)
    flir::SpscMailbox<std::shared_ptr<flir::EOFrameHandle>> mb_frame(1);

    // [EO_TxThread 생성] - 소비자 스레드 (기본 GStreamer 설정 사용)
    // 기본 설정: udp://192.168.0.179:5001, 360x240@30fps
    flir::EO_TxThread tx_thread("EO_TX_Test_Thread", mb_frame);

    // [WakeHandle 생성] - 생산자가 소비자를 깨우기 위한 핸들
    auto wake_handle = tx_thread.create_wake_handle();

    // [소비자 스레드 시작]
    try {
        tx_thread.start();
        std::cout << "[INFO] EO_TxThread started. Streaming to udp://192.168.2.191:5001\n";
    } catch (const std::runtime_error& e) {
        std::cerr << "[ERR] Failed to start EO_TxThread: " << e.what() << "\n";
        return 1;
    }

    // ============================
    //  프레임 생산 및 전송 루프
    // ============================
    
    uint32_t seq = 1;
    int target_fps = 30; // EO 기본 FPS
    int period_ms = 1000 / std::max(1, target_fps);

    for (const auto& file_path : files) {
        if (g_quit.load()) break;
        
        auto t_start = std::chrono::steady_clock::now();

        // === [프레임 생산] JPEG -> cv::Mat(BGR) 로드 ===
        cv::Mat m = cv::imread(file_path, cv::IMREAD_COLOR);
        if (m.empty()) {
            std::cerr << "[WARN] Failed to load image: " << file_path << "\n";
            continue;
        }

        // GStreamer 파이프라인이 BGR 3채널을 기대하므로 타입을 확인/변환합니다.
        if (m.type() != CV_8UC3) {
            std::cerr << "[WARN] Image " << file_path << " is not 8-bit 3 channel. Converting...\n";
            cv::cvtColor(m, m, cv::COLOR_GRAY2BGR);
        }
        
        // 크기 확인 (지금은 리사이즈하지 않고 원본 크기 사용)
        if (seq == 1) {
            std::cout << "[INFO] Using original image size: " << m.cols << "x" << m.rows << std::endl;
        }

        // === EOFrameHandle 꾸러미 생성 ===
        auto h = std::make_shared<MatHandle>();
        h->keep = std::make_shared<cv::Mat>(std::move(m));
        
        auto& owned = h->owned;
        owned.data   = reinterpret_cast<uint8_t*>(h->keep->data);
        owned.width  = h->keep->cols;
        owned.height = h->keep->rows;
        owned.step   = static_cast<int>(h->keep->step);
        
        h->seq = seq++;
        h->ts  = (uint64_t)h->seq * (1000000000ULL / target_fps); // 나노초 단위 타임스탬프

        // [핵심 상호작용] 프레임 생산 -> 소비자 스레드로 전달
        mb_frame.push(h);
        wake_handle->signal(); // EO_TxThread를 깨워서 프레임을 처리하도록 함

        std::cout << "\r[INFO] Sent frame " << h->seq << " / " << files.size() << std::flush;

        // 타겟 FPS 맞추기
        int elapsed_ms = static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(
                          std::chrono::steady_clock::now() - t_start).count());
        int wait_ms = std::max(1, period_ms - elapsed_ms);
        std::this_thread::sleep_for(std::chrono::milliseconds(wait_ms));
    }

    // === 종료 처리 ===
    std::cout << "\n[INFO] End of files. Stopping thread...\n";
    tx_thread.stop();
    tx_thread.join();
    
    std::cout << "[INFO] Done.\n";
    return 0;
}