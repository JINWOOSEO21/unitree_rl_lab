// 정책 관측 조립 — prop 53 과 history 530.
//
// unitree_rl_lab 의 ObservationManager 는 쓸 수 없다. 등록된 term 이 순수
// proprioception 뿐이고(height/scan/lidar 가 없다) 미등록 term 은 throw 한다.
// 그래서 학습 코드(parkour_isaaclab/envs/mdp/observations.py:70-108)의 레이아웃을
// 여기서 직접 조립한다. 값의 출처는 전부 계약(deploy.yaml)과 골든 트레이스다.
//
// prop 53 배치 (index : 내용)
//    0:3   root_ang_vel_b * 0.25      (IMU gyro, body frame)
//    3:5   wrap_to_pi(roll), wrap_to_pi(pitch)     ← projected_gravity 가 아니다
//    5     0
//    6     delta_yaw            (10 Hz 갱신, 배율은 호출자가 이미 적용)
//    7     delta_next_yaw
//    8:10  0, 0
//    10    전진속도 지령
//    11    1.0   (v1.3 규약: 항상 non-flat)
//    12    0.0   (v1.3 규약: 항상 flat=0)
//    13:25 joint_pos - default_joint_pos     (IsaacLab 관절 순서)
//    25:37 joint_vel * 0.05
//    37:49 직전 raw action (지연·클립 이전)
//    49:53 contact_filt - 0.5   (발 4개, IsaacLab 순서 FL/FR/RL/RR)
#pragma once

#include <array>
#include <deque>
#include <stdexcept>
#include <vector>

#include <eigen3/Eigen/Dense>

#include "parkour/contract.h"
#include "parkour/math.h"

namespace parkour
{

constexpr int kNumProp = 53;
constexpr int kNumScan = 132;
constexpr int kHistLen = 10;
constexpr int kNumHist = kNumProp * kHistLen;  // 530
constexpr int kNumJoints = 12;
constexpr int kNumFeet = 4;

/// 한 정책 스텝의 원시 입력. 전부 **IsaacLab 순서**로 들어온다.
struct PropInputs
{
    Eigen::Vector3f gyro_b = Eigen::Vector3f::Zero();          // rad/s
    Eigen::Quaternionf quat_w = Eigen::Quaternionf::Identity();  // (w,x,y,z)
    float delta_yaw = 0.0f;       // 배율 적용 후 값 (HeadingCommand 참조)
    float delta_next_yaw = 0.0f;
    float cmd_vx = 0.0f;
    std::array<float, kNumJoints> joint_pos{};
    std::array<float, kNumJoints> joint_vel{};
    std::array<float, kNumJoints> last_action{};   // raw (지연·클립 이전)
    std::array<bool, kNumFeet> contact{};          // contact_filt (now|prev)
};

/// prop 53 조립 + history 530 유지.
class ObservationBuilder
{
public:
    explicit ObservationBuilder(const Contract& contract) : contract_(contract) {}

    /// prop 53 을 만든다. history 는 건드리지 않는다 (push 는 따로 부른다).
    std::vector<float> build_prop(const PropInputs& in) const
    {
        std::vector<float> p(kNumProp, 0.0f);

        p[0] = in.gyro_b.x() * 0.25f;
        p[1] = in.gyro_b.y() * 0.25f;
        p[2] = in.gyro_b.z() * 0.25f;

        float roll, pitch, yaw;
        euler_xyz_from_quat(in.quat_w, roll, pitch, yaw);
        p[3] = wrap_to_pi(roll);
        p[4] = wrap_to_pi(pitch);

        p[5] = 0.0f;
        p[6] = in.delta_yaw;
        p[7] = in.delta_next_yaw;
        p[8] = 0.0f;
        p[9] = 0.0f;
        p[10] = in.cmd_vx;
        p[11] = 1.0f;   // v1.3: 항상 non-flat
        p[12] = 0.0f;   // v1.3: 항상 flat=0

        for (int i = 0; i < kNumJoints; ++i) {
            p[13 + i] = in.joint_pos[i] - contract_.default_joint_pos[i];
            p[25 + i] = in.joint_vel[i] * 0.05f;
            p[37 + i] = in.last_action[i];
        }
        for (int i = 0; i < kNumFeet; ++i) {
            p[49 + i] = (in.contact[i] ? 1.0f : 0.0f) - 0.5f;
        }
        return p;
    }

    /// 지금 정책에 먹일 history 530.
    ///
    /// **중요**: 학습 코드는 관측을 만들 때 history 버퍼를 먼저 읽고 **그 다음에**
    /// 현재 프레임을 push 한다(observations.py:91-106). 즉 history 는 현재 프레임을
    /// 포함하지 않는 **직전 10프레임**이다. 골든 트레이스로 확인했다 —
    ///   hist[t] == [prop'(t-10) ... prop'(t-1)]   88/88 프레임 정확 일치.
    /// 순진하게 "현재를 넣고 읽으면" 한 프레임씩 밀려서 조용히 틀린다.
    const std::vector<float>& history() const
    {
        if (!primed_) throw std::runtime_error("ObservationBuilder: prime() 를 먼저 불러야 한다");
        return hist_flat_;
    }

    /// 에피소드 시작: history 를 현재 prop 10개 복제로 채운다.
    /// (학습은 episode_length<=1 에서 같은 일을 한다. 다만 학습은 그 스텝의
    ///  관측에 **리셋 이전** 버퍼를 쓰는데, 배포에서 그건 미초기화 값을 먹이는
    ///  것이라 재현하지 않는다 — 진입 시점에 10개 복제로 채우고 시작한다.)
    void prime(const std::vector<float>& prop)
    {
        buf_.clear();
        auto masked = mask_heading(prop);
        for (int i = 0; i < kHistLen; ++i) buf_.push_back(masked);
        primed_ = true;
        flatten();
    }

    /// 한 스텝이 끝난 뒤 현재 prop 을 큐에 넣는다 (가장 오래된 것이 빠진다).
    void push(const std::vector<float>& prop)
    {
        if (!primed_) { prime(prop); return; }
        buf_.pop_front();
        buf_.push_back(mask_heading(prop));
        flatten();
    }

    bool primed() const { return primed_; }
    void reset() { primed_ = false; buf_.clear(); }

private:
    /// history 에 들어가는 값은 6:8(delta_yaw/delta_next_yaw)이 0 으로 지워진다.
    static std::vector<float> mask_heading(const std::vector<float>& prop)
    {
        std::vector<float> q = prop;
        q[6] = 0.0f;
        q[7] = 0.0f;
        return q;
    }

    void flatten()
    {
        hist_flat_.resize(kNumHist);
        int k = 0;
        for (const auto& frame : buf_)
            for (float v : frame) hist_flat_[k++] = v;
    }

    const Contract& contract_;
    std::deque<std::vector<float>> buf_;
    std::vector<float> hist_flat_;
    bool primed_ = false;
};

}  // namespace parkour
