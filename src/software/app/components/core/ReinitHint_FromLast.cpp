#include "components/includes/ReinitHint_FromLast.hpp"
#include <cmath>

namespace flir {

/*
Todo
현재는 그냥 틀만 있는데 나중에 움직이는 방향을 고려해서 살짝 옆으로 옮겨서 다시 타겟팅 위치를 잡아보는 식으로 구현할 예정
이를 위해서 움직이는 방향을 알아야하는데 현재 그 내용이 없음
그 부분이 들어오게 된다면 이걸 사용하고 그게 아니라면 그냥 원래 위치대로 다시 잡아서 재시도 할 예정
*/


cv::Rect2f ReinitHint_FromLast::suggest(const cv::Rect2f& last_box) {
    cv::Rect2f hint = last_box;

    // 1) 속도 기반 1차 예측 (감쇠)
    cv::Point2f dv = vel_ * damp_;
    hint.x += dv.x;
    hint.y += dv.y;

    // 2) 편향(Bias)에 따른 보정 (속도 정보가 약하거나 Auto일 때 유용)
    switch (bias_) {
        case Bias::Left:   hint.x -= step_px_; break;
        case Bias::Right:  hint.x += step_px_; break;
        case Bias::Center:
            if (frame_.width > 0 && frame_.height > 0) {
                const float cx = hint.x + hint.width  * 0.5f;
                const float cy = hint.y + hint.height * 0.5f;
                const float mx = frame_.width  * 0.5f;
                const float my = frame_.height * 0.5f;
                // 중앙 쪽으로 한 스텝
                hint.x += (mx - cx) * 0.1f; // 10%만 이동
                hint.y += (my - cy) * 0.1f;
            }
            break;
        case Bias::Auto:
        default:
            // Auto: 별도 편향 없음 (속도 예측만 사용)
            break;
    }

    // 3) 프레임 경계 내 클램프 (알고리즘 안전성)
    if (frame_.width > 0 && frame_.height > 0) {
        hint.width  = std::max(hint.width,  1.0f);
        hint.height = std::max(hint.height, 1.0f);
        hint.x = clampf(hint.x, 0.0f, std::max(0.0f, (float)frame_.width  - hint.width));
        hint.y = clampf(hint.y, 0.0f, std::max(0.0f, (float)frame_.height - hint.height));
    }

    return hint;
}

} // namespace flir
