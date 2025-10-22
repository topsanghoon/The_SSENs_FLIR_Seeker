#pragma once
#include <optional>
#include <cstdint>
#include <vector>
#include <atomic>
#include <type_traits>
#include <utility>

namespace flir {

// 단일 생산자/단일 소비자(SPSC) 고정 용량 링버퍼 구현.
// - push: 생산자 스레드에서만 호출
// - exchange/has_new/latest_seq: 소비자 스레드에서만 호출
// - 용량 초과 시: 가장 오래된 항목을 "조용히" drop (overwrite) — 실시간 성격에 맞춤
template <typename T>
class SpscMailbox {
public:
    explicit SpscMailbox(size_t capacity = 256) // 기본 사이즈는 256, SpscMailbox(1) 로 생성하면 칸을 수정할 수 있음
    : cap_(capacity), buf_(capacity) {}

    // 생산자: item을 기록
    void push(const T& item) {  // 복사 전달일 경우
        emplace_impl(item);
    }
    void push(T&& item) {       // 기존에 것은 버리고 새로운 것에 전달을 하는 것. 그래서 move 사용
        emplace_impl(std::move(item));
    }

    // 소비자: 마지막으로 본 외부 seq(last_seen)보다 더 "새로운" 항목이 들어왔는지
    // 주의: latest_seq_는 '항목의 seq'를 추적하려 시도(멤버 seq가 있으면 그 값).
    bool has_new(uint32_t last_seen) const {
        return latest_seq_.load(std::memory_order_acquire) > last_seen;
    }

    // 소비자: 새 항목 pop (없으면 std::nullopt)
    std::optional<T> exchange(std::nullptr_t) {
        const size_t r = read_idx_.load(std::memory_order_acquire);
        const size_t w = write_idx_.load(std::memory_order_acquire);
        if (r == w) return std::nullopt; // empty

        T out = std::move(buf_[r % cap_]);
        read_idx_.store(r + 1, std::memory_order_release);
        return out;
    }

    // 소비자: 최신 시퀀스(항목 seq 기반) 조회
    uint32_t latest_seq() const {
        return latest_seq_.load(std::memory_order_acquire);
    }

private:
    // --- seq 특성: T::seq 멤버가 있으면 그 값을 latest_seq_로 사용; 없으면 내부 증가 카운터 사용
    template <typename U>
    static auto has_seq_member(int) -> decltype((void)std::declval<U>().seq, std::true_type{});
    template <typename> static auto has_seq_member(...) -> std::false_type;
    static constexpr bool kHasSeq = decltype(has_seq_member<T>(0))::value;

    uint32_t extract_seq_(const T& item) {
        if constexpr (kHasSeq) {
            // T에 seq 멤버가 있을 때, 그 값을 그대로 사용
            return item.seq;
        } else {
            // 없으면 내부 카운터 사용
            return internal_seq_.fetch_add(1, std::memory_order_relaxed) + 1;
        }
    }

    template <typename U>
    void emplace_impl(U&& item) {
        const size_t w = write_idx_.load(std::memory_order_relaxed);
        const size_t r = read_idx_.load(std::memory_order_acquire);

        // ring에 쓰기
        buf_[w % cap_] = std::forward<U>(item);

        // 용량 초과 시 가장 오래된 것을 drop
        const size_t new_w = w + 1;
        if (new_w - r > cap_) {
            // 하나 밀어내기
            read_idx_.store(new_w - cap_, std::memory_order_release);
        }
        write_idx_.store(new_w, std::memory_order_release);

        // latest_seq_ 갱신 (항목 seq 또는 내부 seq)
        // (주의) 항목에 seq가 없을 때도 “증가” 보장
        uint32_t s = extract_seq_(buf_[(new_w - 1) % cap_]);
        latest_seq_.store(s, std::memory_order_release);
    }

private:
    const size_t                cap_;
    std::vector<T>              buf_;
    std::atomic<size_t>         write_idx_{0};
    std::atomic<size_t>         read_idx_{0};
    std::atomic<uint32_t>       latest_seq_{0};
    std::atomic<uint32_t>       internal_seq_{0};
};

} // namespace flir
