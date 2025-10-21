#include <algorithm>
#include <atomic>
#include <chrono>
#include <filesystem>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>

#include "threads_includes/IR_TrackThread.hpp"
#include "components/includes/IR_Frame.hpp"
#include "components/includes/IR_Preprocessor_LinNorm.hpp"
#include "components/includes/IR_Tracker_MOSSE.hpp"
#include "components/includes/ReinitHint_FromLast.hpp"
#include "components/includes/CsvLoggerIR.hpp"

#include "ipc/event_bus_impl.hpp"
#include "ipc/ipc_types.hpp"
#include "ipc/mailbox.hpp"

namespace fs = std::filesystem;
using namespace std::chrono_literals;

using flir::IR_TrackThread;
using flir::IR_Preprocessor_LinNorm;
using flir::IR_Tracker_MOSSE;
using flir::ReinitHint_FromLast;
using flir::CsvLoggerIR;
using flir::IRFrame16;
using flir::IRFrameHandle;
using flir::UserCmd;
using flir::CmdType;
using flir::Event;
using flir::EventType;
using flir::Topic;
using flir::EventBus;
using flir::SpscMailbox;

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

struct MatHandle : flir::IRFrameHandle {
    std::shared_ptr<cv::Mat> keep;
    flir::IRFrame16 owned{};     // ★ 프레임 메타를 내가 소유
    MatHandle() { p = &owned; }  // base의 p가 owned를 가리키도록
    void release() override { keep.reset(); }
};

static cv::Rect2f make_init_box(const cv::Point& c, const cv::Size& sz, int side=64) {
    int s = std::max(8, std::min(side, std::min(sz.width, sz.height)));
    int x = std::clamp(c.x - s/2, 0, std::max(0, sz.width  - s));
    int y = std::clamp(c.y - s/2, 0, std::max(0, sz.height - s));
    return cv::Rect2f((float)x,(float)y,(float)s,(float)s);
}

static void on_mouse(int event, int x, int y, int, void* userdata) {
    cv::Size* psz = static_cast<cv::Size*>(userdata);
    if (event == cv::EVENT_LBUTTONDOWN && psz) {
        g_click_box = make_init_box({x,y}, *psz, 64);
        g_click_pending = true;
    }
}

static std::vector<std::string> collect_tiffs(const std::string& dir) {
    std::vector<std::string> files;
    for (int i=0;i<=9999;i++) {
        char name[64];
        std::snprintf(name, sizeof(name), "frame_%06d.tiff", i);
        fs::path p = fs::path(dir) / name;
        if (fs::exists(p)) files.push_back(p.string());
        else if (i > 397) break;
    }
    return files;
}

int main(int argc, char** argv) {
    std::string dir = "/home/user/project_dir/The_SSENs_FLIR_Seeker/data/FLIR_Sample";
    if (argc >= 2) dir = argv[1];

    auto files = collect_tiffs(dir);
    if (files.empty()) {
        std::cerr << "[ERR] no TIFFs in " << dir << "\n";
        return 1;
    }
    std::cout << "[INFO] frames=" << files.size() << "\n";

    // ===== pipeline wiring =====
    SpscMailbox<std::shared_ptr<IRFrameHandle>> mb_frame(256);
    SpscMailbox<UserCmd>       mb_click(32);
    EventBus                   bus;

    IR_Tracker_MOSSE           tracker;
    IR_Preprocessor_LinNorm    preproc(1.0f/16383.0f, 0.0f); // 14b→0..1 /(정규화 끄려면 1,0)
    ReinitHint_FromLast        rehint;
    CsvLoggerIR                logger("/tmp/ir_thread_gui.csv");

    flir::IRTrackConfig cfg; // reinit=3, user_req=9

    IR_TrackThread track_thread(mb_frame, mb_click, tracker, preproc, rehint, bus, logger, cfg);

    SpscMailbox<Event> inbox;
    bus.subscribe(Topic::Tracking, &inbox, /*wake*/nullptr);

    track_thread.start();

    cv::namedWindow(kWin, cv::WINDOW_NORMAL);
    cv::resizeWindow(kWin, 960, 720);
    cv::Size last_sz;
    cv::setMouseCallback(kWin, on_mouse, &last_sz);

    uint32_t seq = 1;
    uint32_t last_seq = 0;
    int target_fps = 8;
    int period_ms  = 1000 / std::max(1, target_fps);

    for (size_t i=0; i<files.size(); ++i) {
        auto t_start = std::chrono::steady_clock::now();

        int k = cv::waitKey(1);
        if (k == 'q' || k == 27) { g_quit=true; }
        if (k == ' ') g_paused = !g_paused;
        if (k == '[') { target_fps = std::max(1, target_fps-1); period_ms = 1000/target_fps; }
        if (k == ']') { target_fps = std::min(60, target_fps+1); period_ms = 1000/target_fps; }
        if (g_quit) break;
        if (g_paused) { i = (i>0? i-1:0); std::this_thread::sleep_for(30ms); continue; }

        cv::Mat m = cv::imread(files[i], cv::IMREAD_UNCHANGED);
        if (m.empty()) continue;
        if (m.type()!=CV_16UC1) {
            if (m.channels()==1) m.convertTo(m, CV_16U);
            else cv::cvtColor(m,m,cv::COLOR_BGR2GRAY), m.convertTo(m, CV_16U);
        }
        last_sz = m.size();

        auto sp = std::make_shared<cv::Mat>(std::move(m));
        auto h  = std::make_shared<MatHandle>();
        h->keep = sp;
        h->owned.data = sp->ptr<uint16_t>(0);
        h->owned.width  = sp->cols;
        h->owned.height = sp->rows;
        h->owned.step   = (int)sp->step;
        h->seq = seq++;
        h->ts  = (uint64_t)h->seq * 1000000ULL;
        last_seq = h->seq;     // HUD용

        track_thread.onFrameArrived(h);

        if (g_click_pending.exchange(false)) {
            UserCmd cmd{};
            cmd.type = CmdType::CLICK;
            cmd.box  = g_click_box;
            cmd.seq  = h->seq;
            track_thread.onClickArrived(cmd);
            std::cout << "[GUI] CLICK: " << g_click_box << " (seq="<<cmd.seq<<")\n";
        }

        // 이벤트 드레인
        {
            while (auto ev = inbox.exchange(nullptr)) {
                switch (ev->type) {
                    case EventType::Init: {
                        const auto& x = std::get<flir::InitEvent>(ev->payload);
                        std::lock_guard<std::mutex> lk(g_disp_m);
                        g_last_box = x.box; g_tracking = true; g_fail_streak = 0;
                        break;
                    }
                    case EventType::Track: {
                        const auto& x = std::get<flir::TrackEvent>(ev->payload);
                        std::lock_guard<std::mutex> lk(g_disp_m);
                        g_last_box = x.box; g_last_score = x.score; g_tracking = true; g_fail_streak = 0;
                        break;
                    }
                    case EventType::Lost: {
                        const auto& x = std::get<flir::LostEvent>(ev->payload);
                        std::lock_guard<std::mutex> lk(g_disp_m);
                        g_last_box = x.last_box; g_tracking = false; g_fail_streak++;
                        break;
                    }
                    case EventType::NeedReselect: {
                        std::cout << "[BUS] NeedReselect (fail >= user_req_threshold)\n";
                        break;
                    }
                    default: break;
                }
            }
        }

        // 미리보기
        cv::Mat gray8(sp->rows, sp->cols, CV_8U);
        {
            const size_t N = (size_t)sp->total();
            const uint16_t* src = sp->ptr<uint16_t>(0);
            uint8_t* dst = gray8.ptr<uint8_t>(0);
            for (size_t k2=0;k2<N;++k2) dst[k2] = (uint8_t)(src[k2] >> 6);
        }
        cv::Mat bgr; cv::cvtColor(gray8, bgr, cv::COLOR_GRAY2BGR);

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
+                           last_seq, (int)g_tracking, g_last_score, g_fail_streak, target_fps);
            cv::putText(bgr, buf, {10,22}, cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0,255,0), 2);
        }

        cv::imshow(kWin, bgr);

        int elapsed = (int)std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::steady_clock::now() - t_start).count();
        int wait_ms = std::max(1, period_ms - elapsed);
        cv::waitKey(wait_ms);
    }

    track_thread.stop();
    track_thread.join();
    bus.unsubscribe(&inbox);
    std::cout << "[DONE]\n";
    return 0;
}
