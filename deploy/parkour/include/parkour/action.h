// 액션 후처리 — 1스텝 지연 → 클립 → 스케일 → 기본자세 오프셋.
//
// 학습 코드(parkour_isaaclab/envs/mdp/parkour_actions/joint_actions.py:33-52)를
// 그대로 옮겼다. 요약하면 매 스텝:
//
//     history  = [history[1:], a_t]        // 방금 만든 raw action 을 넣고
//     raw      = history[-1 - delay]       // delay=1 이므로 a_{t-1} 을 꺼내
//     raw      = clip(raw, ±4.8)
//     q_target = raw * 0.25 + q_default    // 이것을 로봇에 보낸다
//
// 즉 **이번 스텝에 실제로 적용되는 것은 직전 스텝의 액션**이다. 관측 obs[37:49] 에
// 들어가는 값은 지연·클립 **이전**의 a_t (history[-1]) 라서 서로 다르다 —
// 둘을 헷갈리면 조용히 한 스텝 어긋난다.
#pragma once

#include <algorithm>
#include <array>
#include <deque>
#include <vector>

#include "parkour/contract.h"
#include "parkour/observation.h"

namespace parkour
{

class ActionPipeline
{
public:
    explicit ActionPipeline(const Contract& c) : c_(c)
    {
        // history 길이는 지연 단계보다 하나 더 있으면 충분하다 (학습은 넉넉히 잡아 둔다).
        const int len = std::max(2, c_.action_delay_steps + 1);
        for (int i = 0; i < len; ++i) history_.push_back(std::array<float, kNumJoints>{});
    }

    /// 정책이 낸 raw action 을 넣고, 이번 스텝에 보낼 q_target 을 돌려준다.
    std::array<float, kNumJoints> push(const std::array<float, kNumJoints>& a_raw)
    {
        history_.pop_front();
        history_.push_back(a_raw);

        // indices = -1 - delay  (학습 코드와 같은 규약)
        const int idx = static_cast<int>(history_.size()) - 1 - c_.action_delay_steps;
        const auto& delayed = history_[static_cast<size_t>(std::max(0, idx))];

        std::array<float, kNumJoints> q{};
        for (int i = 0; i < kNumJoints; ++i) {
            const float clipped = std::clamp(delayed[i], c_.action_clip_lo, c_.action_clip_hi);
            q[i] = clipped * c_.action_scale + c_.default_joint_pos[i];
        }
        return q;
    }

    /// 다음 관측 obs[37:49] 에 넣을 값 = history[-1] = 가장 최근 raw action.
    /// (지연·클립 이전 값이다.)
    const std::array<float, kNumJoints>& last_raw() const { return history_.back(); }

    /// 상태 초기화. 학습의 reset 과 같이 history 를 0 으로 채운다.
    void reset()
    {
        for (auto& h : history_) h.fill(0.0f);
    }

private:
    const Contract& c_;
    std::deque<std::array<float, kNumJoints>> history_;
};

}  // namespace parkour
