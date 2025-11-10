#include "threads_includes/IR_CaptureThread.hpp"

#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <iostream>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <atomic>
#include <vector>

#include "util/common_log.hpp"

namespace flir {

namespace { constexpr const char* TAG = "IR_Cap"; }

// -------------------- Ctor / Dtor --------------------

IR_CaptureThread::IR_CaptureThread(
    std::string name,
    SpscMailbox<std::shared_ptr<IRFrameHandle>>& output_mailbox,
    SpscMailbox<std::shared_ptr<IRFrameHandle>>& out_trk,
    std::unique_ptr<WakeHandle> wake_handle,
    const IRCaptureConfig& config)
    : name_(std::move(name))
    , output_mailbox_(output_mailbox)
    , output_trk_(&out_trk)
    , wake_handle_(std::move(wake_handle))
    , config_(config)
    , spi_fd_(-1)
    // Lepton 2.x 기본: 세그먼트 버퍼는 "세그먼트 1개" 크기
    , segment_buffer_(vospi::PACKETS_PER_SEGMENT * vospi::PAYLOAD_SIZE)
    , frame_buffer_(vospi::FRAME_WIDTH * vospi::FRAME_HEIGHT)
    , packet_buffer_(vospi::PACKET_SIZE)
{}

IR_CaptureThread::~IR_CaptureThread(){
    stop();
    join();
    cleanup_spi();
}

// -------------------- Reset (다단계 복구 시스템) --------------------

void IR_CaptureThread::perform_sync_reset(){
    // Level 1: 동기화 복구만 (하드웨어 건드리지 않음)
    // VoSPI 동기화 문제나 일시적 패킷 순서 오류에 사용
    LOGD(TAG, "Performing sync reset (software only)...");
    // 별도 처리 불필요 - 호출하는 쪽에서 세그먼트 상태만 리셋
}

void IR_CaptureThread::perform_soft_reset(){
    // Level 2: SPI 재시작 (I2C 건드리지 않음)
    // SPI 통신 오류나 드라이버 문제에 사용
    LOGW(TAG, "Performing soft reset (SPI only)...");
    std::lock_guard<std::mutex> lock(spi_mutex_);

    if (spi_fd_ >= 0) {
        ::close(spi_fd_);
        spi_fd_ = -1;
    }

    // SPI만 재초기화 (하드웨어 리부트 없음)
    if (!initialize_spi()) {
        LOGE(TAG, "Failed to reinitialize SPI after soft reset");
    }

    // 짧은 대기 (하드웨어 안정화)
    ::usleep(10000); // 10ms
}

void IR_CaptureThread::perform_safe_reset(){
    // Level 3: 전체 하드웨어 리부트 (최후의 수단)
    // 카메라 완전 정지나 심각한 하드웨어 문제에만 사용
    LOGW(TAG, "Performing full reset (I2C + SPI)...");
    std::lock_guard<std::mutex> lock(spi_mutex_);

    if (spi_fd_ >= 0) {
        ::close(spi_fd_);
        spi_fd_ = -1;
    }

    // I2C reboot (Lepton: /dev/i2c-0, 0x2A)
    int i2c_fd = ::open("/dev/i2c-0", O_RDWR);
    if (i2c_fd >= 0) {
        if (::ioctl(i2c_fd, 0x0703, 0x2a) >= 0) { // I2C_SLAVE
            uint8_t reboot_cmd[4] = {0x08, 0x02, 0x00, 0x00};
            if (::write(i2c_fd, reboot_cmd, 4) == 4) {
                ::usleep(750000); // 750ms - 하드웨어 리부트 대기
            } else {
                LOGE(TAG, "I2C reboot write failed");
            }
        } else {
            LOGE(TAG, "I2C_SLAVE set failed");
        }
        ::close(i2c_fd);
    } else {
        LOGE(TAG, "open(/dev/i2c-0) failed");
    }

    // SPI 재초기화
    reset_requested_.store(false);
    if (!initialize_spi()) {
        LOGE(TAG, "Failed to reinitialize SPI after full reset");
    }

    reset_cv_.notify_one();
}

void IR_CaptureThread::reset_camera(){
    reset_requested_.store(true);
    std::unique_lock<std::mutex> lock(reset_mutex_);
    reset_cv_.wait_for(lock, std::chrono::seconds(5), [this]() {
        return !reset_requested_.load();
    });
}

void IR_CaptureThread::soft_reset_camera(){
    // 외부 호출용 소프트 리셋 (SPI만)
    perform_soft_reset();
}

// -------------------- Start/Stop/Join --------------------

void IR_CaptureThread::start(){
    if (th_.joinable()) return;

    {
        std::lock_guard<std::mutex> lock(spi_mutex_);
        if (!initialize_spi()) {
            LOGE(TAG, "SPI init failed");
            throw std::runtime_error("IR_CaptureThread failed to initialize SPI");
        }
    }

    running_.store(true);
    watchdog_running_.store(true);
    watchdog_thread_ = std::thread(&IR_CaptureThread::watchdog_run, this);
    th_ = std::thread(&IR_CaptureThread::run, this);
}

void IR_CaptureThread::stop(){
    running_.store(false);
    watchdog_running_.store(false);
    reset_cv_.notify_all();
}

void IR_CaptureThread::join(){
    if (th_.joinable()) th_.join();
    if (watchdog_thread_.joinable()) watchdog_thread_.join();
}

// -------------------- Watchdog (지능형 2초) --------------------
//
// 2초마다 확인하되, 에러율과 프레임율을 고려한 지능형 판단
// 단순히 프레임이 없다고 즉시 리셋하지 않고, 시스템 상태를 종합 판단
//

void IR_CaptureThread::watchdog_run(){
    uint64_t last_frames = 0;
    uint64_t last_errors = 0;
    int no_frame_periods = 0;  // 연속으로 프레임이 없는 주기 수

    while (watchdog_running_.load()){
        std::this_thread::sleep_for(std::chrono::seconds(2));  // 2초로 복구
        if (!running_.load()) continue;

        const uint64_t cur_frames = frame_count_.load();
        const uint64_t cur_errors = error_count_.load();
        
        const uint64_t frame_delta = cur_frames - last_frames;
        const uint64_t error_delta = cur_errors - last_errors;
        
        // 프레임이 전혀 없는 경우
        if (frame_delta == 0 && cur_frames > 0) {
            no_frame_periods++;
            
            // 연속 2회(4초) 프레임 없음 → 소프트 리셋
            if (no_frame_periods == 2) {
                LOGW(TAG, "Watchdog: no frames for 4s, high errors → soft reset");
                perform_soft_reset();
                no_frame_periods = 0;
            }
            // 연속 4회(8초) 프레임 없음 → 풀 리셋
            else if (no_frame_periods >= 4) {
                LOGW(TAG, "Watchdog: no frames for 8s → full reset");
                reset_requested_.store(true);
                no_frame_periods = 0;
                
                // 리셋 완료 대기 (최대 3초)
                std::unique_lock<std::mutex> lock(reset_mutex_);
                reset_cv_.wait_for(lock, std::chrono::seconds(3), [this]() {
                    return !reset_requested_.load();
                });
            }
        } else {
            // 프레임이 있으면 카운터 리셋
            no_frame_periods = 0;
            
            // 에러율 체크 (프레임 대비 에러가 너무 높으면)
            if (frame_delta > 0 && error_delta > frame_delta * 3) {
                LOGW(TAG, "Watchdog: high error rate (err=%llu, frames=%llu) → soft reset hint", 
                     (unsigned long long)error_delta, (unsigned long long)frame_delta);
                // 에러율이 높지만 프레임은 나오고 있으니 강제 리셋은 안함
                // 다음 에러 임계값에서 처리하도록 힌트만 남김
            }
        }
        
        last_frames = cur_frames;
        last_errors = cur_errors;
    }
}

// -------------------- SPI I/O --------------------

bool IR_CaptureThread::initialize_spi() {
    if (spi_fd_ >= 0) {
        ::close(spi_fd_);
        spi_fd_ = -1;
    }

    int fd = ::open(config_.spi_device.c_str(), O_RDWR);
    if (fd < 0) {
        LOGE(TAG, "open SPI %s failed", config_.spi_device.c_str());
        return false;
    }

    uint8_t mode = SPI_MODE_3;  // CPOL=1, CPHA=1
    uint8_t bpw  = 8;
    uint32_t spd = config_.spi_speed;

    if (::ioctl(fd, SPI_IOC_WR_MODE, &mode) < 0) {
        LOGE(TAG, "SPI mode set fail");
        ::close(fd);
        return false;
    }
    if (::ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bpw) < 0) {
        LOGE(TAG, "SPI bpw set fail");
        ::close(fd);
        return false;
    }
    if (::ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &spd) < 0) {
        LOGE(TAG, "SPI speed set fail");
        ::close(fd);
        return false;
    }

    spi_fd_ = fd;

    uint32_t actual = 0;
    if (::ioctl(spi_fd_, SPI_IOC_RD_MAX_SPEED_HZ, &actual) >= 0) {
        LOGI(TAG, "SPI ok: req=%uHz act=%uHz",
             (unsigned)config_.spi_speed, (unsigned)actual);
    } else {
        LOGI(TAG, "SPI ok: req=%uHz", (unsigned)config_.spi_speed);
    }

    return true;
}

void IR_CaptureThread::cleanup_spi(){
    std::lock_guard<std::mutex> lk(spi_mutex_);
    if (spi_fd_ >= 0) { ::close(spi_fd_); spi_fd_ = -1; }
}

// -------------------- Main loop --------------------

void IR_CaptureThread::run(){
    LOGI(TAG, "IR capture thread started.");

    auto period = std::chrono::microseconds(1'000'000 / std::max(1, config_.fps));
    auto next_t = std::chrono::steady_clock::now();

    auto log_t0 = std::chrono::steady_clock::now();
    uint64_t last_f=0, last_e=0, last_d=0;

    // 상수 로깅(디버깅 편의)
    LOGI(TAG, "VoSPI cfg: SEG=%d PPK=%d LINES=%d W=%d H=%d PAYLOAD=%d",
         vospi::SEGMENTS_PER_FRAME, vospi::PACKETS_PER_SEGMENT,
         vospi::LINES_PER_SEGMENT, vospi::FRAME_WIDTH,
         vospi::FRAME_HEIGHT, vospi::PAYLOAD_SIZE);

    while (running_.load()){
        auto frame_t0 = std::chrono::steady_clock::now();

        if (reset_requested_.load()){
            perform_safe_reset();
            next_t = std::chrono::steady_clock::now();
            continue;
        }

        try{
            if (capture_vospi_frame()){
                auto h = create_frame_handle();
                if (h){
                    push_frame_routed_(h);
                    frame_count_.fetch_add(1);
                } else {
                    error_count_.fetch_add(1);
                }
            } else {
                error_count_.fetch_add(1);
            }
        } catch(const std::exception& e){
            LOGE(TAG, "Capture exception: %s", e.what());
            error_count_.fetch_add(1);
        }

        // 1초 통계
        auto now = std::chrono::steady_clock::now();
        if (now - log_t0 >= std::chrono::seconds(1)){
            uint64_t f = frame_count_.load(), e = error_count_.load(), d = discard_count_.load();
            LOGI(TAG, "IR stats: fps=%llu err+=%llu disc+=%llu (total f=%llu e=%llu d=%llu)",
                 (unsigned long long)(f-last_f), (unsigned long long)(e-last_e), (unsigned long long)(d-last_d),
                 (unsigned long long)f, (unsigned long long)e, (unsigned long long)d);
            last_f=f; last_e=e; last_d=d; log_t0=now;
        }

        next_t += period;
        auto sleep_until = std::max(next_t, frame_t0 + std::chrono::microseconds(1000));
        std::this_thread::sleep_until(sleep_until);
    }

    LOGI(TAG, "IR capture thread stopped.");
}

// -------------------- VoSPI (지능형 다단계 복구) --------------------
//
// - 동기화 에러: 단순 세그먼트 재시작 (250회 → sync reset)
// - 디스카드/텔레메트리: SPI 재시작 (1500회 → soft reset)  
// - 심각한 동기화 실패: 전체 리셋 (3000회 → hard reset)
// - SPI 통신 에러: 즉시 SPI 재시작 (10회)
// - 성공 시 에러 카운터 점진적 감소로 자연 복구
//

bool IR_CaptureThread::capture_vospi_frame(){
    for (int seg=0; seg<vospi::SEGMENTS_PER_FRAME; ++seg){
        if (!capture_segment(seg)) return false;
        if (seg < vospi::SEGMENTS_PER_FRAME-1)
            std::this_thread::sleep_for(std::chrono::microseconds(10));
    }
    reconstruct_frame();
    return true;
}

bool IR_CaptureThread::capture_segment(int seg_id){
    // 다단계 에러 임계값 (점진적 대응)
    constexpr int SYNC_RESET_THRESHOLD = 250;   // Level 1: 동기화 리셋
    constexpr int SOFT_RESET_THRESHOLD = 1500;  // Level 2: SPI 리셋  
    constexpr int HARD_RESET_THRESHOLD = 3000;  // Level 3: 전체 리셋
    
    int sync_errors = 0;    // 동기화 관련 에러
    int discard_count = 0;  // 디스카드 패킷 수
    int spi_errors = 0;     // SPI 통신 에러

    // Lepton 3.x 대비: 세그먼트 시작 오프셋(바이트). Lepton 2.5면 SEGMENTS_PER_FRAME=1.
    const int seg_base = seg_id * (vospi::PACKETS_PER_SEGMENT * vospi::PAYLOAD_SIZE);

    int expected_line = 0; // 0..59
    int packets = 0;       // 유효 라인 수

    while (running_.load() && packets < vospi::PACKETS_PER_SEGMENT) {
        // 패킷 버퍼 크기 보장
        if (packet_buffer_.size() < vospi::PACKET_SIZE) {
            LOGE(TAG, "CRITICAL: packet buffer too small! size=%zu need=%d",
                 packet_buffer_.size(), vospi::PACKET_SIZE);
            return false;
        }

        if (!read_vospi_packet(packet_buffer_.data())) {
            LOGE(TAG, "SPI read failed");
            if (++spi_errors >= 10) {  // SPI 에러는 빠르게 대응
                LOGW(TAG, "Multiple SPI failures → soft reset");
                perform_soft_reset();
                return false;  // 세그먼트 재시도
            }
            continue;
        }

        // 0xFxxx (discard/텔레메트리) → 소프트 에러 (하드웨어는 정상)
        if (is_discard_packet(packet_buffer_.data())) {
            discard_count_.fetch_add(1);
            ::usleep(100);
            if (++discard_count >= SOFT_RESET_THRESHOLD) {
                LOGW(TAG, "Too many discards (%d) → soft reset (SPI only)", discard_count);
                perform_soft_reset();
                // 세그먼트 상태 초기화 후 다시 시도
                discard_count = 0; sync_errors = 0; expected_line = 0; packets = 0;
            }
            continue;
        }

        const int line = get_packet_line_number(packet_buffer_.data());

        // 텔레메트리 라인(예: line==60) → 소프트 에러
        if (line >= vospi::LINES_PER_SEGMENT) {
            ::usleep(50);
            if (++discard_count >= SOFT_RESET_THRESHOLD) {
                LOGW(TAG, "Too many telemetry skips (%d) → soft reset (SPI only)", discard_count);
                perform_soft_reset();
                discard_count = 0; sync_errors = 0; expected_line = 0; packets = 0;
            }
            continue;
        }

        // 라인 불일치/비정상 → 동기화 에러 (가장 흔한 케이스)
        if (line < 0 || line != expected_line) {
            ++sync_errors;
            
            // 단계별 대응
            if (sync_errors >= HARD_RESET_THRESHOLD) {
                LOGW(TAG, "Critical sync errors (%d) → full reset (I2C + SPI)", sync_errors);
                perform_safe_reset();
                return false;  // 세그먼트 완전 실패
            } else if (sync_errors >= SOFT_RESET_THRESHOLD) {
                LOGW(TAG, "Many sync errors (%d) → soft reset (SPI only)", sync_errors);
                perform_soft_reset();
                sync_errors = 0; discard_count = 0; expected_line = 0; packets = 0;
                continue;
            } else if (sync_errors >= SYNC_RESET_THRESHOLD) {
                LOGD(TAG, "Sync errors (%d) → resync segment", sync_errors);
                perform_sync_reset();
                sync_errors = 0; expected_line = 0; packets = 0;
                ::usleep(500);
                continue;
            }
            
            // 소수 에러는 단순 재동기화
            expected_line = 0;
            packets = 0;
            ::usleep(500);
            continue;
        }

        // 안전 memcpy
        const int off = seg_base + packets * vospi::PAYLOAD_SIZE;
        if (off < 0 ||
            off + vospi::PAYLOAD_SIZE > static_cast<int>(segment_buffer_.size())) {
            LOGE(TAG, "CRITICAL: segment buffer memcpy overflow! seg=%d off=%d payload=%d size=%zu",
                 seg_id, off, vospi::PAYLOAD_SIZE, segment_buffer_.size());
            return false;
        }
        if (packet_buffer_.size() < 4 + vospi::PAYLOAD_SIZE) {
            LOGE(TAG, "CRITICAL: packet buffer underflow! size=%zu need=%d",
                 packet_buffer_.size(), 4 + vospi::PAYLOAD_SIZE);
            return false;
        }

        std::memcpy(&segment_buffer_[off], &packet_buffer_[4], vospi::PAYLOAD_SIZE);

        ++packets;        // 유효 라인 누적
        ++expected_line;  // 다음 기대 라인
        
        // 성공적인 패킷 수신 시 에러 카운터들을 점진적으로 감소
        if (sync_errors > 0) sync_errors = std::max(0, sync_errors - 1);
        if (discard_count > 0) discard_count = std::max(0, discard_count - 1);
        if (spi_errors > 0) spi_errors = 0;  // SPI 에러는 즉시 리셋
    }

    return (packets == vospi::PACKETS_PER_SEGMENT);
}

bool IR_CaptureThread::read_vospi_packet(uint8_t* pkt){
    std::lock_guard<std::mutex> lock(spi_mutex_);

    if (spi_fd_ < 0){
        LOGE(TAG, "SPI fd invalid");
        return false;
    }
    if (!pkt) {
        LOGE(TAG, "CRITICAL: NULL packet buffer passed to read_vospi_packet");
        return false;
    }

    struct spi_ioc_transfer tr{};
    tr.tx_buf = 0;
    tr.rx_buf = reinterpret_cast<uintptr_t>(pkt);
    tr.len = vospi::PACKET_SIZE;
    tr.speed_hz = config_.spi_speed;
    tr.bits_per_word = 8;
    tr.cs_change = 1;
    tr.delay_usecs = static_cast<__u16>(config_.spi_delay_usecs);

    int r = ::ioctl(spi_fd_, SPI_IOC_MESSAGE(1), &tr);
    if (r < 0) {
        LOGE(TAG, "SPI ioctl failed: errno=%d", errno);
        return false;
    }
    return true;
}

// 패킷→프레임(16U) 재구성
void IR_CaptureThread::reconstruct_frame(){
    for (int seg=0; seg<vospi::SEGMENTS_PER_FRAME; ++seg){
        for (int line=0; line<vospi::LINES_PER_SEGMENT; ++line){
            const int frame_line = seg*vospi::LINES_PER_SEGMENT + line;
            const int seg_off = (seg*vospi::PACKETS_PER_SEGMENT*vospi::PAYLOAD_SIZE)
                              + (line*vospi::PAYLOAD_SIZE);
            for (int px=0; px<vospi::PIXELS_PER_LINE; ++px){
                const int frame_idx = frame_line*vospi::FRAME_WIDTH + px;
                const int seg_idx   = seg_off + (px*2);

                if (seg_idx + 1 >= static_cast<int>(segment_buffer_.size())) {
                    LOGE(TAG, "CRITICAL: segment buffer overflow! seg_idx=%d+1 size=%zu seg=%d line=%d px=%d",
                         seg_idx, segment_buffer_.size(), seg, line, px);
                    return;
                }
                if (frame_idx >= static_cast<int>(frame_buffer_.size())) {
                    LOGE(TAG, "CRITICAL: frame buffer overflow! frame_idx=%d size=%zu frame_line=%d px=%d",
                         frame_idx, frame_buffer_.size(), frame_line, px);
                    return;
                }

                const uint16_t v = (static_cast<uint16_t>(segment_buffer_[seg_idx])<<8)
                                 | static_cast<uint16_t>(segment_buffer_[seg_idx+1]);
                frame_buffer_[frame_idx] = v;
            }
        }
    }
}

// -------------------- Frame handle & routing --------------------

std::shared_ptr<IRFrameHandle> IR_CaptureThread::create_frame_handle(){
    if (frame_buffer_.empty()) {
        return nullptr;
    }

    auto handle = std::make_shared<IRMatHandle>();

    cv::Mat temp_frame(vospi::FRAME_HEIGHT, vospi::FRAME_WIDTH, CV_16UC1, frame_buffer_.data());
    if (temp_frame.empty()) {
        return nullptr;
    }

    handle->keep = std::make_shared<cv::Mat>(temp_frame.clone());
    if (!handle->keep || handle->keep->empty() || !handle->keep->isContinuous()) {
        return nullptr;
    }
    if (!handle->keep->data) {
        return nullptr;
    }

    auto& owned = handle->owned;
    owned.data   = reinterpret_cast<uint16_t*>(handle->keep->data);
    owned.width  = handle->keep->cols;
    owned.height = handle->keep->rows;
    owned.step   = static_cast<int>(handle->keep->step);

    if (!owned.data) {
        return nullptr;
    }
    if (owned.width != vospi::FRAME_WIDTH || owned.height != vospi::FRAME_HEIGHT) {
        return nullptr;
    }

    handle->p   = &owned;
    handle->seq = sequence_.fetch_add(1);
    handle->ts  = get_timestamp_ns();
    return handle;
}

void IR_CaptureThread::push_frame_routed_(const std::shared_ptr<IRFrameHandle>& h){
    // TX로 항상
    output_mailbox_.push(h);
    if (wake_handle_) wake_handle_->signal();

    // Terminal일 때만 트래커로 라우팅
    auto phase = GuidanceState::phase().load(std::memory_order_relaxed);
    const bool route_to_track = (phase == GuidancePhase::Terminal);
    if (!route_to_track) return;

    if (track_sink_) { track_sink_(h); return; }
    if (output_trk_) {
        output_trk_->push(h);
        if (track_wake_) track_wake_->signal();
    }
}

// -------------------- Packet helpers --------------------

inline bool IR_CaptureThread::is_sync_packet(const uint8_t* p){
    return (p[0]==0x00) && (p[1]==0x00);
}

// 0xFxxx 헤더(텔레메트리/디스카드) 검출
inline bool IR_CaptureThread::is_discard_packet(const uint8_t* p){
    return ((p[0] & 0x0F) == 0x0F);
}

// 두 번째 바이트(라인 번호)
inline int IR_CaptureThread::get_packet_line_number(const uint8_t* p){
    return static_cast<int>(p[1]);
}

uint64_t IR_CaptureThread::get_timestamp_ns(){
    auto now = std::chrono::high_resolution_clock::now();
    return std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
}

} // namespace flir
