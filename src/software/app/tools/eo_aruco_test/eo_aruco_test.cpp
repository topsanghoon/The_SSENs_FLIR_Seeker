// eo_aruco_test.cpp (EventInbox 의존 제거 + bbox→box 수정 + 뷰어는 로컬 감지)
// ---------------------------------------------------------------

#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <optional>
#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#include "threads_includes/EO_ArUcoThread.hpp"
#include "ipc/mailbox.hpp"
#include "ipc/event_bus_impl.hpp"
#include "ipc/event_bus.hpp"
// #include "ipc/ipc_types.hpp" // EO_ArUcoThread 내부에서 이미 사용

#include "components/includes/EO_Frame.hpp"
#include "components/includes/CsvLoggerAru.hpp"

#include "util/common_log.hpp"
#include "main_config.hpp"

using namespace std::chrono_literals;

namespace flir {

// 테스트용 EOFrameHandle (cv::Mat 생명주기 보존)
struct EO_MatHandle : EOFrameHandle {
    std::shared_ptr<cv::Mat> keep;
    FrameBGR8 owned{};
    EO_MatHandle() { p = &owned; }
    void release() override { keep.reset(); }
    void retain()  override {}
};

// BGR → GRAY8
class SimpleArucoPreproc : public IArucoPreprocessor {
public:
    void run(const EOFrameHandle& in, cv::Mat& pf_gray8) override {
        const FrameBGR8& f = *in.p;
        cv::Mat src(f.height, f.width, CV_8UC3, (void*)f.data, f.step);
        cv::cvtColor(src, pf_gray8, cv::COLOR_BGR2GRAY);
    }
};

// 4x4_50 딕셔너리 감지기
class SimpleArucoDetector : public IArucoDetector {
public:
    SimpleArucoDetector() {
        dict_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
        params_ = cv::aruco::DetectorParameters::create();
    }
    std::vector<Detection> detect(const cv::Mat& pf_gray8) override {
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;
        cv::aruco::detectMarkers(pf_gray8, dict_, corners, ids, params_);
        std::vector<Detection> out;
        out.reserve(ids.size());
        for (size_t i = 0; i < ids.size(); ++i) {
            Detection d;
            d.id = ids[i];
            std::array<cv::Point2f,4> cs{};
            for (int k=0; k<4 && k<(int)corners[i].size(); ++k) cs[k] = corners[i][k];
            d.corners = cs;
            d.bbox = cv::boundingRect(corners[i]);
            return out;
        }
        return out;
    }
private:
    cv::Ptr<cv::aruco::Dictionary> dict_;
    cv::Ptr<cv::aruco::DetectorParameters> params_;
};

} // namespace flir

static std::atomic<bool> g_quit{false};
static void on_signal(int) { g_quit.store(true); }

static bool open_camera(cv::VideoCapture& cap, int index)
{
    if (cap.open(index, cv::CAP_V4L2)) { std::cout << "[INFO] opened camera via CAP_V4L2\n"; return true; }
    if (cap.open(index, cv::CAP_ANY))  { std::cout << "[INFO] opened camera via CAP_ANY\n";  return true; }
    return false;
}
static bool open_camera_gst(cv::VideoCapture& cap, int cam_index)
{
    std::ostringstream p;
    p << "v4l2src device=/dev/video" << cam_index << " ! videoconvert ! appsink";
    if (cap.open(p.str(), cv::CAP_GSTREAMER)) {
        std::cout << "[INFO] opened camera via GStreamer pipeline\n";
        return true;
    }
    return false;
}

int main(int argc, char** argv)
{
    int cam_index = 0;
    bool show = false;
    for (int i=1;i<argc;i++){
        std::string a = argv[i];
        if (a == "--show") show = true;
        else cam_index = std::stoi(a);
    }

    auto cfg = std::make_shared<flir::AppConfig>();
    const int W   = cfg->eo_tx.frame.width;
    const int H   = cfg->eo_tx.frame.height;
    const int FPS = std::max(1, cfg->eo_tx.fps);

    std::signal(SIGINT,  on_signal);
    std::signal(SIGTERM, on_signal);

    cv::VideoCapture cap;
    bool opened = open_camera(cap, cam_index);
    if (!opened) opened = open_camera_gst(cap, cam_index);
    if (!opened) { std::cerr << "[ERR] cannot open camera index " << cam_index << "\n"; return 1; }

    cap.set(cv::CAP_PROP_FRAME_WIDTH,  W);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, H);
    cap.set(cv::CAP_PROP_FPS,          FPS);
    std::cout << "[INFO] actual size=" << (int)cap.get(cv::CAP_PROP_FRAME_WIDTH)
              << "x" << (int)cap.get(cv::CAP_PROP_FRAME_HEIGHT)
              << " fps=" << (int)cap.get(cv::CAP_PROP_FPS) << "\n";

    // ----- 파이프라인 구성(버스/메일박스/스레드) -----
    flir::EventBus bus;
    flir::SpscMailbox<std::shared_ptr<flir::EOFrameHandle>> mb_eo(2);

    const std::string csv_path = cfg->paths.csv_root + "/aruco_track.csv";
    flir::CsvLoggerAru csv_logger(csv_path);

    flir::SimpleArucoPreproc preproc;
    flir::SimpleArucoDetector detector;

    flir::EO_ArUcoThread aruco_thr(mb_eo, preproc, detector, bus, csv_logger);
    aruco_thr.start();

    std::cout << "[INFO] webcam=/dev/video" << cam_index
              << " size=" << W << "x" << H << " fps=" << FPS << "\n";
    if (show) std::cout << "[INFO] viewer ON (--show)\n";
    std::cout << "[INFO] press Ctrl+C to stop.\n";

    uint32_t seq = 1;
    const int period_ms = 1000 / FPS;

    // (옵션) 뷰어용 로컬 감지기/전처리 (EventBus 구독 불필요)
    cv::Ptr<cv::aruco::Dictionary> v_dict;
    cv::Ptr<cv::aruco::DetectorParameters> v_params;
    if (show) {
        v_dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
        v_params = cv::aruco::DetectorParameters::create();
        cv::namedWindow("EO ArUco Viewer", cv::WINDOW_AUTOSIZE);
    }

    while (!g_quit.load()) {
        auto t0 = std::chrono::steady_clock::now();

        cv::Mat frame_bgr;
        if (!cap.read(frame_bgr) || frame_bgr.empty()) {
            std::cerr << "[WARN] camera frame empty\n";
            std::this_thread::sleep_for(10ms);
            continue;
        }
        if (frame_bgr.cols != W || frame_bgr.rows != H) {
            cv::resize(frame_bgr, frame_bgr, cv::Size(W, H));
        }

        // EO_ArUcoThread에 투입
        auto h = std::make_shared<flir::EO_MatHandle>();
        h->keep = std::make_shared<cv::Mat>(frame_bgr.clone()); // 스레드용 복사
        h->owned.data   = reinterpret_cast<uint8_t*>(h->keep->data);
        h->owned.width  = h->keep->cols;
        h->owned.height = h->keep->rows;
        h->owned.step   = static_cast<int>(h->keep->step);
        h->seq = seq++;
        h->ts  = (uint64_t)h->seq * (1000000000ULL / FPS);
        mb_eo.push(h);

        // (옵션) 로컬 뷰어: 간단 감지 + 오버레이 (EventInbox 미사용)
        if (show) {
            cv::Mat gray;
            cv::cvtColor(frame_bgr, gray, cv::COLOR_BGR2GRAY);

            std::vector<int> ids;
            std::vector<std::vector<cv::Point2f>> corners;
            cv::aruco::detectMarkers(gray, v_dict, corners, ids, v_params);

            for (size_t i=0;i<ids.size();++i) {
                cv::Rect box = cv::boundingRect(corners[i]);
                cv::rectangle(frame_bgr, box, cv::Scalar(0,255,0), 2);
                for (int k=0;k<4 && k<(int)corners[i].size(); ++k) {
                    cv::circle(frame_bgr, corners[i][k], 3, cv::Scalar(0,0,255), -1);
                }
                cv::putText(frame_bgr, "id="+std::to_string(ids[i]),
                            cv::Point(box.x, std::max(0, box.y-5)),
                            cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0,255,0), 1, cv::LINE_AA);
            }

            cv::imshow("EO ArUco Viewer", frame_bgr);
            int k = cv::waitKey(1);
            if (k == 'q' || k == 27) g_quit.store(true);
        }

        int elapsed = (int)std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::steady_clock::now() - t0).count();
        int wait_ms = std::max(1, period_ms - elapsed);
        std::this_thread::sleep_for(std::chrono::milliseconds(wait_ms));
    }

    std::cout << "[STOP] stopping...\n";
    aruco_thr.stop();
    aruco_thr.join();
    if (show) cv::destroyAllWindows();
    std::cout << "[DONE]\n";
    return 0;
}
