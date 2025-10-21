#include <algorithm>
#include <atomic>
#include <chrono>
#include <filesystem>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <iomanip>  // setprecision용

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>

// === Thread 파이프라인에서 쓰는 인터페이스/타입들 ===
#include "threads_includes/IR_TrackThread.hpp"           // IR_TrackThread (소비자)
#include "components/includes/IR_Frame.hpp"              // IRFrameHandle/IRFrame16
#include "components/includes/IR_Preprocessor_LinNorm.hpp" // IPreprocessor 구현 (16U→32F)
#include "components/includes/IR_Tracker_MOSSE.hpp"      // ITrackerStrategy 구현 (MOSSE)
#include "components/includes/CsvLoggerIR.hpp"           // CSV 로깅

#include "ipc/event_bus_impl.hpp"   // EventBus (발행/구독)
#include "ipc/ipc_types.hpp"        // Event, UserCmd 등 DTO
#include "ipc/mailbox.hpp"          // SpscMailbox<T>

namespace fs = std::filesystem;
using namespace std::chrono_literals;

// (HUD/입출력용 전역 상태)
static const char* kWin = "IR Thread GUI";
static std::atomic<bool> g_quit{false};
static std::atomic<bool> g_paused{false};
static std::atomic<bool> g_click_pending{false};
static cv::Rect2f g_click_box{};
static std::mutex g_disp_m;
static cv::Rect2f g_last_box{};
static float      g_last_score = 0.0f;
static bool       g_tracking   = false;
static int        g_fail_streak= 0;
static std::string g_evt_text;

// === 프레임 핸들 구현(샘플 뷰어가 보유하는 Mat를 release()에서 해제) ===
// - IR_TrackThread는 IRFrameHandle의 포인터/시퀀스/타임스탬프만 소비.
// - 실제 버퍼 소유권은 MatHandle::keep(shared_ptr<Mat>)가 보유하고, release()에서 reset().
struct MatHandle : flir::IRFrameHandle {
    std::shared_ptr<cv::Mat> keep;
    flir::IRFrame16 owned{};     // 프레임 메타(포인터/폭/높이/스텝)를 내부에서 보유
    MatHandle() { p = &owned; }  // base 포인터(p)가 내가 가진 owned를 가리키도록 연결
    void release() override { keep.reset(); } // 소비자 쪽에서 frame 사용 끝났을 때 호출됨
};

// === (클릭) 초기 ROI 박스 생성: GUI 좌표 → 정사각형 박스 ===
static cv::Rect2f make_init_box(const cv::Point& c, const cv::Size& sz, int side=64) {
    int s = std::max(8, std::min(side, std::min(sz.width, sz.height)));
    int x = std::clamp(c.x - s/2, 0, std::max(0, sz.width  - s));
    int y = std::clamp(c.y - s/2, 0, std::max(0, sz.height - s));
    return cv::Rect2f((float)x,(float)y,(float)s,(float)s);
}

// === GUI 마우스 콜백: 클릭 → 초기화 박스 후보 설정 ===
// - 여기서 바로 스레드에 넣지 않고, 메인 루프에서 “클릭 대기 플래그”를 확인해 UserCmd 발행.
static void on_mouse(int event, int x, int y, int, void* userdata) {
    cv::Size* psz = static_cast<cv::Size*>(userdata);
    if (event == cv::EVENT_LBUTTONDOWN && psz) {
        g_click_box = make_init_box({x,y}, *psz, 64);
        g_click_pending = true;
    }
}

// === 샘플 TIFF 파일 수집 ===
static std::vector<std::string> collect_tiffs(const std::string& dir) {
    std::vector<std::string> files;
    for (int i=0;i<=9999;i++) {
        char name[64];
        std::snprintf(name, sizeof(name), "frame_%06d.tiff", i);
        fs::path p = fs::path(dir) / name;
        if (fs::exists(p)) files.push_back(p.string());
        else if (i > 397) break; // 샘플 데이터 상한(필요 시 조정)
    }
    return files;
}

int main(int argc, char** argv) {
    // ★ 입력 프레임 소스(샘플 데이터 디렉토리)
    std::string dir = "/home/user/project_dir/The_SSENs_FLIR_Seeker/data/FLIR_Sample";
    if (argc >= 2) dir = argv[1];

    auto files = collect_tiffs(dir);
    if (files.empty()) {
        std::cerr << "[ERR] no TIFFs in " << dir << "\n";
        return 1;
    }
    std::cout << "[INFO] frames=" << files.size() << "\n";

    // ============================
    //  파이프라인 와이어링(핵심)
    // ============================

    // [프레임 메일박스]  (생산자: 이 GUI 재생 루프 / 소비자: IR_TrackThread)
    // - 타입: shared_ptr<IRFrameHandle>
    // - 생산자는 push(), 소비자는 exchange(nullptr)
    flir::SpscMailbox<std::shared_ptr<flir::IRFrameHandle>> mb_frame(1);

    // [클릭 메일박스]    (생산자: 이 GUI 재생 루프 / 소비자: IR_TrackThread)
    flir::SpscMailbox<flir::UserCmd>       mb_click(32);

    // [이벤트 버스]      (발행: IR_TrackThread / 구독: 이 GUI 루프)
    flir::EventBus                         bus;

    // [전처리/트래커/로거] (IR_TrackThread 협력자)
    // - 전처리: GRAY16 → GRAY32F(정규화; 14bit/16383)
    // - 트래커: MOSSE (cv::legacy::TrackerMOSSE)
    // - 로거: /tmp/ir_thread_gui.csv 로 이벤트 기록
    flir::IR_Preprocessor           preproc(1.0f/16383.0f, 0.0f);
    flir::IR_Tracker_MOSSE          tracker;
    flir::CsvLoggerIR               logger("/tmp/ir_thread_gui.csv");

    // [reinit 제거] 정책:
    // - IR_TrackThread 내부에서 “재초기화”를 하지 않고,
    //   누적 실패가 user_req_threshold(기본 9) 도달 시 NeedReselect 이벤트만 발생.

    // [스레드 설정값]
    // - reinit_threshold는 의미없음(사용 안 함). user_req_threshold만 유효.
    flir::IRTrackConfig cfg; // { .reinit_threshold=3, .user_req_threshold=9 }

    // [IR_TrackThread 생성]
    // - 소비자 스레드: 프레임/클릭 메일박스를 읽고, 전처리→init/update→이벤트 발행
    // - reinit 제거: 내부에서 fail_streak >= 9 되면 NeedReselect만 1회 발행(디바운스)
    flir::IR_TrackThread track_thread(mb_frame, mb_click, tracker, preproc, bus, logger, cfg);

    // [GUI 측 이벤트 구독 Inbox]
    // - Topic::Tracking에 대해 구독하여 Init/Track/Lost/NeedReselect 이벤트 수신
    flir::SpscMailbox<flir::Event> inbox;
    bus.subscribe(flir::Topic::Tracking, &inbox, /*wake*/nullptr);

    // [소비자 스레드 시작]
    track_thread.start();

    // ============================
    //  GUI/HUD & 재생 루프(생산자)
    // ============================

    cv::namedWindow(kWin, cv::WINDOW_NORMAL);
    cv::resizeWindow(kWin, 960, 720);
    cv::Size last_sz;
    cv::setMouseCallback(kWin, on_mouse, &last_sz);

    uint32_t seq = 1;         // 프레임 ID(메일박스 seq로도 쓰임)
    uint32_t last_seq = 0;    // HUD 표시에 사용
    int target_fps = 8;
    int period_ms  = 1000 / std::max(1, target_fps);

    for (size_t i=0; i<files.size(); ++i) {
        auto t_start = std::chrono::steady_clock::now();

        // 키 입력 (q: 종료, space: 일시정지, [ / ]: FPS 조절)
        int k = cv::waitKey(1);
        if (k == 'q' || k == 27) { g_quit=true; }
        if (k == ' ') g_paused = !g_paused;
        if (k == '[') { target_fps = std::max(1, target_fps-1); period_ms = 1000/target_fps; }
        if (k == ']') { target_fps = std::min(60, target_fps+1); period_ms = 1000/target_fps; }
        if (g_quit) break;
        if (g_paused) { i = (i>0? i-1:0); std::this_thread::sleep_for(30ms); continue; }

        // === [프레임 생산] TIFF → cv::Mat(16UC1) 로드 ===
        cv::Mat m = cv::imread(files[i], cv::IMREAD_UNCHANGED);
        if (m.empty()) continue;
        if (m.type()!=CV_16UC1) {
            if (m.channels()==1) m.convertTo(m, CV_16U);
            else cv::cvtColor(m,m,cv::COLOR_BGR2GRAY), m.convertTo(m, CV_16U);
        }
        last_sz = m.size();

        // === IRFrameHandle 꾸러미 만들어 메일박스로 push ===
        // - 소비자는 shared_ptr<IRFrameHandle>을 받아 사용하고 release() 호출
        auto sp = std::make_shared<cv::Mat>(std::move(m));
        auto h  = std::make_shared<MatHandle>();
        h->keep = sp;
        h->owned.data   = sp->ptr<uint16_t>(0);
        h->owned.width  = sp->cols;
        h->owned.height = sp->rows;
        h->owned.step   = (int)sp->step;
        h->seq = seq++;
        h->ts  = (uint64_t)h->seq * 1000000ULL; // 데모용 타임스탬프
        last_seq = h->seq;

        // [핵심 상호작용 #1] 프레임 생산 → 소비자 스레드로 전달
        // - mb_frame.push(h) + (스레드 내부 condvar signal)
        track_thread.onFrameArrived(h);

        // === 클릭이 들어왔으면 UserCmd 생성해 클릭 메일박스로 push ===
        if (g_click_pending.exchange(false)) {
            flir::UserCmd cmd{};
            cmd.type = flir::CmdType::CLICK;
            cmd.box  = g_click_box; // 초기화 박스
            cmd.seq  = h->seq;      // 관련 프레임 ID (로깅 편의)
            // [핵심 상호작용 #2] 클릭 생산 → 소비자 스레드로 전달
            track_thread.onClickArrived(cmd);
            std::cout << "[GUI] CLICK: " << g_click_box << " (seq="<<cmd.seq<<")\n";
        }

        // === [핵심 상호작용 #3] EventBus에서 소비자 스레드가 발행한 이벤트 드레인 ===
        // - Init/Track/Lost/NeedReselect를 수신하여 HUD 상태 갱신/로그 출력
        {
            auto print_rect = [](const cv::Rect2f& r) {
                std::cout << "box=("
                        << std::fixed << std::setprecision(1)
                        << r.x << "," << r.y << "," << r.width << "," << r.height << ")";
                std::cout.unsetf(std::ios::floatfield);
            };

            while (auto ev = inbox.exchange(nullptr)) {
                switch (ev->type) {
                    case flir::EventType::Init: {
                        const auto& x = std::get<flir::InitEvent>(ev->payload);
                        {
                            std::lock_guard<std::mutex> lk(g_disp_m);
                            g_last_box = x.box; g_tracking = true; g_fail_streak = 0;
                            g_evt_text = "EVT: INIT";
                        }
                        std::cout << "[EVT][Init] seq=" << x.frame_seq
                                << " ts=" << x.ts << " ";
                        print_rect(x.box);
                        std::cout << "\n";
                        break;
                    }

                    case flir::EventType::Track: {
                        const auto& x = std::get<flir::TrackEvent>(ev->payload);
                        {
                            std::lock_guard<std::mutex> lk(g_disp_m);
                            g_last_box = x.box; g_last_score = x.score; g_tracking = true; g_fail_streak = 0;
                            g_evt_text = "EVT: TRACK";
                        }
                        std::cout << "[EVT][Track] seq=" << x.frame_seq
                                << " ts=" << x.ts
                                << " score=" << std::fixed << std::setprecision(3) << x.score << " ";
                        print_rect(x.box);
                        std::cout.unsetf(std::ios::floatfield);
                        std::cout << "\n";
                        break;
                    }

                    case flir::EventType::Lost: {
                        const auto& x = std::get<flir::LostEvent>(ev->payload);
                        {
                            std::lock_guard<std::mutex> lk(g_disp_m);
                            g_last_box = x.last_box; g_tracking = false; g_fail_streak++;
                            g_evt_text = "EVT: LOST";
                        }
                        std::cout << "[EVT][Lost] seq=" << x.frame_seq
                                << " ts=" << x.ts << " ";
                        print_rect(x.last_box);
                        std::cout << " fail_streak=" << g_fail_streak << "\n";
                        break;
                    }

                    case flir::EventType::NeedReselect: {
                        {
                            std::lock_guard<std::mutex> lk(g_disp_m);
                            g_evt_text = "EVT: NEED_RESELECT";
                        }
                        std::cout << "[EVT][NeedReselect] fail_streak >= user_req_threshold\n";
                        break;
                    }

                    default:
                        std::cout << "[EVT][Unknown]\n";
                        break;
                }
            }
        }


        // === 미리보기 렌더링(16U → 8U) ===
        cv::Mat gray8(sp->rows, sp->cols, CV_8U);
        {
            const size_t N = (size_t)sp->total();
            const uint16_t* src = sp->ptr<uint16_t>(0);
            uint8_t* dst = gray8.ptr<uint8_t>(0);
            for (size_t k2=0;k2<N;++k2) dst[k2] = (uint8_t)(src[k2] >> 6);
        }
        cv::Mat bgr; cv::cvtColor(gray8, bgr, cv::COLOR_GRAY2BGR);

        // HUD
        {
            std::lock_guard<std::mutex> lk(g_disp_m);
            if (g_last_box.width>1 && g_last_box.height>1) {
                cv::Scalar col = g_tracking ? cv::Scalar(0,255,0) : cv::Scalar(0,0,255);
                cv::rectangle(bgr, g_last_box, col, 2);
            }
            if (!g_tracking && g_fail_streak>0) {
                cv::putText(bgr, "TRACK LOST", {10,46}, cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0,165,255), 2);
            }
            if (g_click_pending) {
                cv::rectangle(bgr, g_click_box, cv::Scalar(0,165,255), 1);
            }
            char buf[256];
            std::snprintf(buf, sizeof(buf), "seq=%u  tracking=%d  score=%.2f  fail=%d  fps=%d",
                          last_seq, (int)g_tracking, g_last_score, g_fail_streak, target_fps);
            cv::putText(bgr, buf, {10,22}, cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0,255,0), 2);
        }

        cv::imshow(kWin, bgr);

        // 타겟 FPS 맞추기
        int elapsed = (int)std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::steady_clock::now() - t_start).count();
        int wait_ms = std::max(1, period_ms - elapsed);
        cv::waitKey(wait_ms);
    }

    // === 종료 처리 ===
    track_thread.stop();
    track_thread.join();
    bus.unsubscribe(&inbox);
    std::cout << "[DONE]\n";
    return 0;
}
