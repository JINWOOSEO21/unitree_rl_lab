// 각도 변환 — IsaacLab isaaclab/utils/math.py 와 **같은 식**을 쓴다.
//
// 왜 직접 쓰는가: Eigen 의 eulerAngles() 는 축 순서·분기 규약이 달라 값이 미묘하게
// 어긋난다. 관측 obs[3:5] 가 roll/pitch 라서 여기서 어긋나면 정책 입력이 조용히
// 틀어진다. 학습 코드와 한 줄씩 대응시켜 두는 편이 안전하다.
#pragma once

#include <cmath>

#include <eigen3/Eigen/Dense>

namespace parkour
{

/// IsaacLab wrap_to_pi: 각도를 [-pi, pi] 로. pi 의 홀수배는 부호를 유지한다.
inline float wrap_to_pi(float angle)
{
    constexpr float kPi = static_cast<float>(M_PI);
    const float wrapped = std::fmod(angle + kPi, 2.0f * kPi);
    // fmod 는 음수 입력에 음수를 돌려주므로 한 번 더 올린다 (파이썬 % 와 맞춘다).
    const float positive = wrapped < 0.0f ? wrapped + 2.0f * kPi : wrapped;
    if (positive == 0.0f && angle > 0.0f) return kPi;
    return positive - kPi;
}

/// IsaacLab euler_xyz_from_quat (XYZ extrinsic). quat 은 (w, x, y, z).
/// 반환값은 이미 (-pi, pi] 범위다 (atan2 / asin).
inline void euler_xyz_from_quat(const Eigen::Quaternionf& q,
                                float& roll, float& pitch, float& yaw)
{
    const float w = q.w(), x = q.x(), y = q.y(), z = q.z();

    const float sin_roll = 2.0f * (w * x + y * z);
    const float cos_roll = 1.0f - 2.0f * (x * x + y * y);
    roll = std::atan2(sin_roll, cos_roll);

    float sin_pitch = 2.0f * (w * y - z * x);
    if (std::fabs(sin_pitch) >= 1.0f) {
        pitch = std::copysign(static_cast<float>(M_PI) / 2.0f, sin_pitch);
    } else {
        pitch = std::asin(sin_pitch);
    }

    const float sin_yaw = 2.0f * (w * z + x * y);
    const float cos_yaw = 1.0f - 2.0f * (y * y + z * z);
    yaw = std::atan2(sin_yaw, cos_yaw);
}

inline float yaw_from_quat(const Eigen::Quaternionf& q)
{
    float r, p, y;
    euler_xyz_from_quat(q, r, p, y);
    return y;
}

}  // namespace parkour
