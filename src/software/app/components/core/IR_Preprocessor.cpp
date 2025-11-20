#include "components/includes/IR_Preprocessor.hpp"
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>

namespace flir {

// k, alpha는 필요에 따라 조절(아래 값은 비교적 보수적이고 안정적)
// 출력은 최종적으로 8bit 그레이(CV_8UC1)
static inline void normalize_stable_ema_u8(const cv::Mat& src16, cv::Mat& out8u) {
    CV_Assert(src16.type() == CV_16UC1);

    // 전역 상태 없이도 동작하게 정적 로컬 상태 사용
    static float ema_mean = 10000.f;
    static float ema_std  =  1200.f;
    static float ema_lo   =  8000.f;
    static float ema_hi   = 12000.f;

    // 1) 통계 추정
    cv::Scalar m, s;
    cv::meanStdDev(src16, m, s);
    const float mean = (float)m[0];
    const float stdv = (float)s[0];

    // 2) EMA로 느리게 추종 (프레임 요동에 둔감)
    const float alpha = 0.05f; // 0.02~0.08 범위 추천
    ema_mean = (1.f - alpha)*ema_mean + alpha*mean;
    ema_std  = (1.f - alpha)*ema_std  + alpha*std::max(50.f, stdv);

    // 3) 로버스트 구간 [lo, hi] 계산 (μ±kσ)
    const float k = 2.0f;     // 1.5~2.5 추천
    float lo = ema_mean - k*ema_std;
    float hi = ema_mean + k*ema_std;

    // 4) 안전 클램프 (센서 범위 가드 + 최소 폭)
    lo = std::max(0.f, lo);
    hi = std::min(65535.f, hi);
    if (hi - lo < 256.f) { // 너무 좁으면 MOSSE/KCF가 못 느낄 수 있음
        const float mid = 0.5f*(lo + hi);
        lo = mid - 128.f;
        hi = mid + 128.f;
        lo = std::max(0.f, lo);
        hi = std::min(65535.f, hi);
    }

    // EMA로 범위도 완만하게 (히스테리시스)
    ema_lo = (1.f - alpha)*ema_lo + alpha*lo;
    ema_hi = (1.f - alpha)*ema_hi + alpha*hi;
    lo = ema_lo;
    hi = ema_hi;

    // 5) [lo,hi] → [0,1] 선형 맵 + 소프트 클램프 (float32 내부 버퍼)
    const float inv = 1.f / std::max(1.f, (hi - lo));
    cv::Mat tmp32;
    src16.convertTo(tmp32, CV_32F, inv, -lo*inv);
    cv::threshold(tmp32, tmp32, 0, 0, cv::THRESH_TOZERO);
    cv::threshold(tmp32, tmp32, 1, 1, cv::THRESH_TRUNC);

    // 6) 소프트 고주파 강조(언샤프닝, 아주 약하게)
    cv::Mat blur;
    cv::GaussianBlur(tmp32, blur, cv::Size(0,0), 1.0, 1.0);
    const float beta = 0.35f; // 0.2~0.5
    cv::Mat high = tmp32 - blur;
    tmp32 = tmp32 + beta * high;
    cv::threshold(tmp32, tmp32, 0, 0, cv::THRESH_TOZERO);
    cv::threshold(tmp32, tmp32, 1, 1, cv::THRESH_TRUNC);

    // (옵션) 감마 0.9로 중간톤 살짝 끌어올릴 수도 있음
    // cv::pow(tmp32, 0.9, tmp32);

    // 7) [0,1] float → [0,255] 8bit로 최종 변환
    tmp32.convertTo(out8u, CV_8U, 255.0);
}

void IR_Preprocessor::run(const IRFrame16& in16, cv::Mat& outMat) {
    cv::Mat src(in16.height, in16.width, CV_16UC1, in16.data, in16.step);

    // 기존 고정 스케일 대신, 안정 정규화 + 약한 고주파 강조 → 8bit
    normalize_stable_ema_u8(src, outMat);
}

} // namespace flir