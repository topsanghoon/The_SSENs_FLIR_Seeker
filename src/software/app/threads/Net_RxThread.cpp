// Net_RxThread.cpp  (CSV unified: start/stop + per-dgram loop + CLICK_RX/SD_RX)
#include "threads_includes/Net_RxThread.hpp"

#include <arpa/inet.h>
#include <cerrno>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cstring>
#include <sstream>
#include <string_view>
#include <algorithm>
#include <cctype>
#include <iostream>

// ✅ 공통 유틸
#include "util/common_log.hpp"   // LOGI/LOGW/LOGE...
#include "util/time_util.hpp"    // ScopedTimerMs
#include "util/csv_sink.hpp"     // CSV_LOG_SIMPLE
#include "util/telemetry.hpp"

namespace flir {

namespace { constexpr const char* TAG = "Net_Rx"; }

Net_RxThread::Net_RxThread(std::string name,
                           AppConfigPtr cfg,
                           SpscMailbox<UserCmd>& click_out,
                           SpscMailbox<SelfDestructCmd>& sd_out,
                           IEventBus& bus)
    : name_(std::move(name))
    , cfg_(std::move(cfg))
    , out_click_(click_out)
    , out_sd_(sd_out)
    , bus_(bus)                               // ★
{
    rxbuf_.resize(std::max<size_t>(cfg_->net_rx.buffer_size, 1024));
}

Net_RxThread::~Net_RxThread() {
    stop();
    join();
    close_socket_();
}

void Net_RxThread::start() {
    if (running_.exchange(true)) return;
    if (!init_socket_()) {
        running_.store(false);
        throw std::runtime_error("Net_RxThread socket init failed");
    }
    CSV_LOG_SIMPLE("Net.Rx", "THREAD_START", 0, 0,0,0,0, "");
    th_ = std::thread(&Net_RxThread::run_, this);
}

void Net_RxThread::stop() {
    if (!running_.load()) return;
    running_.store(false);
    // poll 깨우기용 dummy datagram (loopback)
    if (sock_ >= 0) {
        sockaddr_in self{}; self.sin_family = AF_INET; self.sin_port = htons(bind_port_); self.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        ::sendto(sock_, "", 0, 0, (sockaddr*)&self, sizeof(self));
    }
}

void Net_RxThread::join() {
    if (th_.joinable()) th_.join();
    CSV_LOG_SIMPLE("Net.Rx", "THREAD_STOP", 0, 0,0,0,0, "");
}

bool Net_RxThread::init_socket_() {
    close_socket_();

    bind_port_ = cfg_->net_rx.port;
    sock_ = ::socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC, 0);
    if (sock_ < 0) { LOGE(TAG, "socket: %s", strerror(errno)); return false; }
    own_sock_ = true;

    sockaddr_in a{}; a.sin_family = AF_INET; a.sin_port = htons(bind_port_); a.sin_addr.s_addr = htonl(INADDR_ANY);
    if (::bind(sock_, (sockaddr*)&a, sizeof(a)) != 0) {
        LOGE(TAG, "bind :%u failed: %s", bind_port_, strerror(errno));
        ::close(sock_); sock_ = -1; own_sock_ = false; return false;
    }

    LOGI(TAG, "listening UDP :%u (timeout=%dms, buf=%zu, click_box=%.1f)",
         bind_port_, cfg_->net_rx.timeout_ms, rxbuf_.size(), cfg_->net_rx.click_box_size);
    return true;
}

void Net_RxThread::close_socket_() {
    if (own_sock_ && sock_ >= 0) { ::close(sock_); sock_ = -1; own_sock_ = false; }
}

void Net_RxThread::run_() {
    const int timeout_ms = cfg_->net_rx.timeout_ms;
    uint32_t pkt_seq = 0;

    while (running_.load()) {
        pollfd pfd{ .fd = sock_, .events = POLLIN, .revents = 0 };
        int r = ::poll(&pfd, 1, timeout_ms);
        if (!running_.load()) break;

        if (r < 0) {
            if (errno == EINTR) continue;
            LOGE(TAG, "poll: %s", strerror(errno));
            break;
        }
        if (r == 0) continue;

        if (pfd.revents & POLLIN) {
            sockaddr_in src{}; socklen_t sl = sizeof(src);
            const ssize_t n = ::recvfrom(sock_, rxbuf_.data(), rxbuf_.size(), 0, (sockaddr*)&src, &sl);
            if (n <= 0) continue;

            const uint32_t seq = ++pkt_seq;
            double loop_ms = 0.0;
            CSV_LOG_SIMPLE("Net.Rx", "LOOP_BEGIN", seq, (double)n, 0,0,0, "");
            {
                ScopedTimerMs t(loop_ms);
                handle_datagram_(rxbuf_.data(), static_cast<size_t>(n), src);
            }
            CSV_LOG_SIMPLE("Net.Rx", "LOOP_END",   seq, loop_ms, 0,0,0, "");
        }
    }

    LOGI(TAG, "run() exit");
}

static inline void trim_in_place(std::string& s) {
    auto issp = [](unsigned char c){ return std::isspace(c); };
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), [&](char c){ return !issp(c); }));
    s.erase(std::find_if(s.rbegin(), s.rend(), [&](char c){ return !issp(c); }).base(), s.end());
}
static inline std::string to_lower(std::string s){
    for (char& c: s) if (c>='A' && c<='Z') c = c - 'A' + 'a';
    return s;
}

void Net_RxThread::handle_datagram_(const uint8_t* data, size_t len, const sockaddr_in&) {
    if (len == 0) return;

    // ── 0) 바이너리 자폭: [0x01][level]
    SelfDestructCmd sd{};
    if (parse_cmd_sd_bin_(data, len, sd)) {
        out_sd_.push(sd);
        CSV_LOG_SIMPLE("Net.Rx","SD_RX", sd.seq, (double)sd.level, 0,0,0, "");
        // ★ 이벤트버스로도 발행 → ControlThread 깨움
        bus_.push(Event{ EventType::SelfDestruct, SelfDestructEvent{ sd.level, sd.seq } }, Topic::User);
        return;
    }

    // ── 1) 바이너리 클릭: [0x00][f32_be x][f32_be y]
    if (len >= 9 && data[0] == 0x00) {
        int32_t xi_be, yi_be;
        std::memcpy(&xi_be, data+1, 4);
        std::memcpy(&yi_be, data+5, 4);
        const uint32_t xi = ntohl(static_cast<uint32_t>(xi_be));
        const uint32_t yi = ntohl(static_cast<uint32_t>(yi_be));
        float fx, fy; std::memcpy(&fx, &xi, 4); std::memcpy(&fy, &yi, 4);

        if (std::isfinite(fx) && std::isfinite(fy)) {
            const float b = cfg_->net_rx.click_box_size;
            UserCmd cmd{};
            cmd.type = CmdType::CLICK;
            cmd.box  = { fx - b*0.5f, fy - b*0.5f, b, b };
            cmd.seq  = ++click_seq_;

            out_click_.push(cmd);
            CSV_LOG_SIMPLE("Net.Rx", "CLICK_RX", cmd.seq, cmd.box.x, cmd.box.y, cmd.box.width, cmd.box.height, "");
            // ★ 이벤트버스로도 발행 → IR_TrackThread 깨움(onClickArrived 내부 notify)
            bus_.push(Event{ EventType::UserClick, UserClickEvent{ cmd.box, cmd.seq } }, Topic::User);
            return;
        }
        // 값 비정상 → 텍스트 경로로 폴백
    }

    // ── 2) 텍스트 명령
    std::string msg(reinterpret_cast<const char*>(data), len);
    trim_in_place(msg);
    if (msg.empty()) return;

    // 2-1) 자폭 텍스트: "SD" / "SD <level>" / "SELF" / "SELF_DESTRUCT [level]"
    if (parse_cmd_sd_text_(msg, sd)) {
        out_sd_.push(sd);
        CSV_LOG_SIMPLE("Net.Rx", "SD_RX", sd.seq, (double)sd.level, 0,0,0, "");
        // ★ 이벤트버스 발행
        bus_.push(Event{ EventType::SelfDestruct, SelfDestructEvent{ sd.level, sd.seq } }, Topic::User);
        return;
    }

    // 2-2) 클릭 텍스트: "CLICK x y" (포맷 유연, parse_cmd_click_이 좌표만 뽑음)
    UserCmd click{};
    if (parse_cmd_click_(msg, click)) {
        out_click_.push(click);
        CSV_LOG_SIMPLE("Net.Rx", "CLICK_RX", click.seq, click.box.x, click.box.y, click.box.width, click.box.height, "");
        // ★ 이벤트버스 발행
        bus_.push(Event{ EventType::UserClick, UserClickEvent{ click.box, click.seq } }, Topic::User);
        return;
    }

    // 3) 그 외는 조용히 드롭
    LOGW(TAG, "unrecognized msg ...");
}


// 숫자 2개만 뽑아내는 보조 함수(공백/콤마/콜론/괄호 등은 구분자로 취급)
static bool extract_two_floats(const std::string& s, float& x, float& y) {
    // 구분자들을 공백으로 통일
    std::string t; t.reserve(s.size());
    for (char c : s) {
        if (c==',' || c==';' || c==':' || c=='[' || c==']' || c=='(' || c==')' || std::isspace((unsigned char)c))
            t.push_back(' ');
        else
            t.push_back(c);
    }
    std::istringstream iss(t);
    double a, b;
    // 앞쪽에 문자열 토큰이 섞여도 숫자 2개만 건져오도록 스캔
    std::string tok;
    bool gotA=false, gotB=false;
    while (iss) {
        if (iss.peek()==EOF) break;
        if (std::isdigit(iss.peek()) || iss.peek()=='-' || iss.peek()=='+') {
            if (!gotA && (iss >> a)) { gotA = true; continue; }
            if ( gotA && !gotB && (iss >> b)) { gotB = true; break; }
            // 실패 시 상태 복구
            if (!iss) { iss.clear(); iss >> tok; }
        } else {
            iss >> tok; // 버림
        }
    }
    if (gotA && gotB) { x = static_cast<float>(a); y = static_cast<float>(b); return true; }
    return false;
}

bool Net_RxThread::parse_cmd_click_(const std::string& msg, UserCmd& out) {
    // 1) 트림 + 소문자 복사본으로 CLICK 접두어 유무 확인
    std::string s = msg;
    trim_in_place(s);
    std::string sl = to_lower(s);

    // 2) "click" 접두어 있으면 제거(뒤에 콜론/콤마/공백 허용)
    if (sl.rfind("click", 0) == 0) {
        // 원본 s에서 'CLICK' 길이만큼 제거
        s.erase(0, 5);
        trim_in_place(s);
        // 앞에 콜론/콤마가 바로 오면 제거
        while (!s.empty() && (s[0]==':' || s[0]==',')) { s.erase(0,1); trim_in_place(s); }
    }

    // 3) 남은 문자열에서 숫자 2개만 뽑기 (형식 자유)
    float fx, fy;
    if (!extract_two_floats(s, fx, fy)) return false;

    const float b = cfg_->net_rx.click_box_size;
    out.type = CmdType::CLICK;
    out.box  = {fx - b*0.5f, fy - b*0.5f, b, b};
    out.seq  = ++click_seq_;
    return true;
}


bool Net_RxThread::parse_cmd_sd_text_(const std::string& msg, SelfDestructCmd& out) {
    // 허용 토큰: SD / sd / SELF / self / SELF_DESTRUCT / self_destruct
    std::istringstream iss(msg);
    std::string head; if (!(iss >> head)) return false;

    auto eq = [&](const char* s){
        if (head.size() != std::strlen(s)) return false;
        for (size_t i=0;i<head.size();++i){
            char a=head[i], b=s[i];
            if (a>='A'&&a<='Z') a = a-'A'+'a';
            if (b>='A'&&b<='Z') b = b-'A'+'a';
            if (a!=b) return false;
        }
        return true;
    };

    if (!(eq("sd") || eq("self") || eq("self_destruct"))) return false;

    int level = 1; // 기본 레벨 1
    if (iss.good()) {
        int tmp;
        if (iss >> tmp) level = tmp;
    }

    out.level = level;
    out.seq   = ++sd_seq_;
    return true;
}

bool Net_RxThread::parse_cmd_sd_bin_(const uint8_t* data, size_t len, SelfDestructCmd& out) {
    if (len == 2 && data[0] == 0x01) {
        out.level = static_cast<int>(data[1]);
        out.seq   = ++sd_seq_;
        return true;
    }
    return false;
}

} // namespace flir
