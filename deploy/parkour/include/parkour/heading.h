// 조향 명령 — 학습의 delta_yaw 자리를 배포에서 무엇으로 채울 것인가.
//
// 학습에서 obs[6:8] 은 파쿠르 목표점에서 나왔다:
//     delta_yaw      = target_yaw - wrap_to_pi(yaw)      (10 Hz 갱신)
//     target_yaw     = atan2(목표점 - 로봇위치)          (월드 각도)
//     delta_next_yaw = 다음 목표점에 대해 같은 것
// 즉 정책이 받는 것은 **방향 오차**다 (회전 명령이 아니다). 정책 입력에 각속도
// 명령 자체가 없으므로, 조향 수단은 이 값 하나뿐이다.
//
// 배포에는 목표점이 없다. 그래서 목표 방향을 하나 만들어 두고 그 오차를 먹인다:
//
//     target_yaw += rx * kTurnRate * dt          // 스틱을 적분
//     delta_yaw   = kHeadingScale * wrap_to_pi(target_yaw - yaw)
//     delta_next_yaw = delta_yaw                 // "다음 목표" 가 없다
//
// 이러면 스틱을 놓았을 때 target_yaw 가 월드에 고정되므로 로봇은 **한 직선을
// 유지**한다 (단순 직진이 아니라, 밀려나면 되돌아온다).
//
// 키보드 q/e 는 시뮬레이터 쪽에서 **순간 펄스**로 낸다 (누르면 0.2 s 동안 rx=±1).
// 그래서 한 번 누르면 약 kTurnRate*0.2s = 15° 씩 꺾이고, 계속 누르면 이어서 돈다.
// 실제 아날로그 스틱은 밀고 있는 동안 계속 값이 들어오므로 같은 코드가 그대로
// 연속 선회로 동작한다 — 배포 코드는 한 가지 규칙만 안다.
//
// delta_next_yaw 를 delta_yaw 와 같게 두는 것은 play.py --fixed_heading 과 같다
// (scripts/rsl_rl/play.py: obs[:,6] = obs[:,7] = 1.5 * delta).
#pragma once

#include <algorithm>
#include <cmath>

#include "parkour/math.h"

namespace parkour
{

class HeadingCommand
{
public:
    /// 스틱 최대치에서의 목표방향 회전 속도 [rad/s].
    /// 0.2 s 펄스에 15° 가 되도록 잡았다 (15° / 0.2 s = 75°/s).
    static constexpr float kTurnRate = 1.309f;  // 75 deg/s

    /// obs[6:8] 에 넣을 때의 배율.
    /// **학습 관측은 배율 없는 raw delta_yaw 였다.** 그런데 play.py --fixed_heading
    /// 은 1.5 를 곱한다(depth encoder 시절 `1.5*yaw` 스케일의 잔재). 재생에서 실제로
    /// 쓰이던 값이 1.5 쪽이라 배포도 그것을 따른다 — 사용자 결정 2026-09-04.
    /// 조향 감도가 1.5 배 달라지는 값이므로 바꿀 때는 반드시 PLAY 로 확인할 것.
    static constexpr float kHeadingScale = 1.5f;

    /// 상태 진입 시 호출. 지금 향한 방향을 목표로 삼는다 → delta_yaw = 0 (직진).
    void reset(float yaw_now)
    {
        target_yaw_ = yaw_now;
        started_ = true;
    }

    /// EM tick(10 Hz)마다 호출. rx 는 조이스틱 오른쪽 스틱 x [-1, 1].
    /// 부호: rx > 0 이면 target_yaw 가 **감소**(시계방향 = 오른쪽)한다.
    /// 유니트리 조이스틱 규약(오른쪽 = +rx)과 월드 yaw(반시계 = +)를 맞춘 것이다.
    void update(float rx, float yaw_now, float dt)
    {
        if (!started_) { reset(yaw_now); return; }
        target_yaw_ = wrap_to_pi(target_yaw_ - std::clamp(rx, -1.0f, 1.0f) * kTurnRate * dt);
        yaw_now_ = yaw_now;
    }

    /// obs[6] 에 넣을 값 (배율 적용 후).
    float delta_yaw() const
    {
        return kHeadingScale * wrap_to_pi(target_yaw_ - yaw_now_);
    }

    /// obs[7]. "다음 목표" 가 없으므로 같은 값을 쓴다.
    float delta_next_yaw() const { return delta_yaw(); }

    float target_yaw() const { return target_yaw_; }

    /// yaw 만 갱신 (정책 스텝마다 부르면 조향이 매 스텝 따라간다).
    /// 학습은 10 Hz 로만 갱신하므로 기본 경로에서는 쓰지 않는다.
    void set_yaw(float yaw_now) { yaw_now_ = yaw_now; }

private:
    float target_yaw_ = 0.0f;
    float yaw_now_ = 0.0f;
    bool started_ = false;
};

}  // namespace parkour
