// 3-4 게이트: 배포 C++ 가 조립한 관측·액션이 IsaacLab 실측과 같은가.
//
// 골든 트레이스(IsaacLab 100 스텝)의 **원시 상태**를 먹여 prop 53 / history 530 을
// 다시 만들고, 기록된 값과 대조한다. 마지막으로 ONNX 를 실제로 돌려 액션까지 본다.
// 여기를 통과하면 남은 위험은 "실기에서 그 원시 상태를 제대로 읽는가" 뿐이다.
//
//   ./test_obs_golden <trace.bin> <deploy.yaml> <policy.onnx>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <string>
#include <vector>

#include "isaaclab/algorithms/algorithms.h"
#include "parkour/action.h"
#include "parkour/contract.h"
#include "parkour/observation.h"

using namespace parkour;

namespace
{

struct Trace
{
    int T = 0;
    std::vector<std::vector<float>> prop, scan, hist, actions;
    std::vector<std::vector<float>> joint_pos, joint_vel, gyro, quat;
    std::vector<std::vector<float>> contact_now, contact_prev;
};

bool read_block(std::ifstream& f, int T, int dim, std::vector<std::vector<float>>& out)
{
    out.assign(T, std::vector<float>(dim));
    for (int t = 0; t < T; ++t) {
        f.read(reinterpret_cast<char*>(out[t].data()), dim * sizeof(float));
        if (!f) return false;
    }
    return true;
}

bool load_trace(const std::string& path, Trace& tr)
{
    std::ifstream f(path, std::ios::binary);
    if (!f) { std::printf("트레이스를 열 수 없다: %s\n", path.c_str()); return false; }
    char magic[4];
    f.read(magic, 4);
    if (std::memcmp(magic, "PKGT", 4) != 0) { std::printf("magic 불일치\n"); return false; }
    f.read(reinterpret_cast<char*>(&tr.T), 4);
    return read_block(f, tr.T, 53, tr.prop) && read_block(f, tr.T, 132, tr.scan) &&
           read_block(f, tr.T, 530, tr.hist) && read_block(f, tr.T, 12, tr.actions) &&
           read_block(f, tr.T, 12, tr.joint_pos) && read_block(f, tr.T, 12, tr.joint_vel) &&
           read_block(f, tr.T, 3, tr.gyro) && read_block(f, tr.T, 4, tr.quat) &&
           read_block(f, tr.T, 12, tr.contact_now) && read_block(f, tr.T, 12, tr.contact_prev);
}

/// |d| <= atol + rtol*|want| — 절대오차만 쓰면 float32 반올림이 FAIL 로 보인다.
struct Cmp
{
    double worst_abs = 0.0;
    int bad = 0, n = 0;
    int worst_idx = -1;
    double worst_got = 0, worst_want = 0;

    void add(float got, float want, int idx, float atol = 1e-5f, float rtol = 1e-4f)
    {
        ++n;
        const double d = std::fabs(static_cast<double>(got) - want);
        if (d > worst_abs) { worst_abs = d; worst_idx = idx; worst_got = got; worst_want = want; }
        if (d > atol + rtol * std::fabs(static_cast<double>(want))) ++bad;
    }
};

}  // namespace

int main(int argc, char** argv)
{
    if (argc < 4) {
        std::printf("사용법: %s <trace.bin> <deploy.yaml> <policy.onnx>\n", argv[0]);
        return 2;
    }
    Trace tr;
    if (!load_trace(argv[1], tr)) return 1;
    std::printf("트레이스 %d 프레임\n\n", tr.T);

    const Contract c = Contract::load(argv[2]);
    ObservationBuilder obs(c);
    ActionPipeline act(c);

    int fails = 0;

    // ---------------------------------------------------------------- [1] prop
    // 우리가 만들 수 없는 값(조향 6:7, 속도 지령 10)은 기록값을 그대로 주입한다.
    // 배포에서 그 자리는 HeadingCommand / 조이스틱이 채우므로 여기서 볼 대상이 아니다.
    Cmp cp;
    for (int t = 0; t < tr.T; ++t) {
        PropInputs in;
        in.gyro_b = Eigen::Vector3f(tr.gyro[t][0], tr.gyro[t][1], tr.gyro[t][2]);
        in.quat_w = Eigen::Quaternionf(tr.quat[t][0], tr.quat[t][1], tr.quat[t][2], tr.quat[t][3]);
        in.delta_yaw = tr.prop[t][6];
        in.delta_next_yaw = tr.prop[t][7];
        in.cmd_vx = tr.prop[t][10];
        for (int i = 0; i < kNumJoints; ++i) {
            in.joint_pos[i] = tr.joint_pos[t][i];
            in.joint_vel[i] = tr.joint_vel[t][i];
            // obs[37:49] 는 직전 스텝의 raw action 이다 (지연·클립 이전).
            in.last_action[i] = (t == 0) ? tr.prop[0][37 + i] : tr.actions[t - 1][i];
        }
        for (int k = 0; k < kNumFeet; ++k) {
            auto norm3 = [](const std::vector<float>& v, int k) {
                return std::sqrt(v[3 * k] * v[3 * k] + v[3 * k + 1] * v[3 * k + 1] +
                                 v[3 * k + 2] * v[3 * k + 2]);
            };
            in.contact[k] = (norm3(tr.contact_now[t], k) > 2.0f) ||
                            (norm3(tr.contact_prev[t], k) > 2.0f);
        }
        const auto p = obs.build_prop(in);
        for (int i = 0; i < kNumProp; ++i) cp.add(p[i], tr.prop[t][i], i);

        if (t == 0) obs.prime(p); else obs.push(p);
    }
    std::printf("[1] prop 53 조립          최대오차 %.3e  불일치 %d/%d  → %s\n",
                cp.worst_abs, cp.bad, cp.n, cp.bad ? "FAIL" : "PASS");
    if (cp.bad) {
        std::printf("    최악: idx %d  got %.6f  want %.6f\n",
                    cp.worst_idx, cp.worst_got, cp.worst_want);
        ++fails;
    }

    // ------------------------------------------------------------- [2] history
    // 다시 처음부터 돌리며, **읽는 시점**을 학습과 맞춘다:
    // 관측을 만들 때는 버퍼를 먼저 읽고 그 다음에 현재 프레임을 push 한다.
    //
    // 비교 시작점이 t=11 인 이유: IsaacLab 은 episode_length<=1 인 동안 버퍼를
    // "현재 프레임 10개 복제" 로 **다시** 채운다(observations.py:98-105). 트레이스의
    // t=0,1 이 그 창에 걸려 있어 t<=10 까지는 버퍼 앞쪽에 prop'(1) 이 중복으로
    // 남는다. 배포에는 에피소드가 없고 진입 시 한 번만 채우므로 이 재충전은
    // 재현하지 않는다 — 그 창을 벗어난 뒤부터 대조한다.
    // (확인: 재충전을 반영한 예측은 t=9 부터 전부 0/530 으로 일치했다.)
    obs.reset();
    Cmp chh;
    for (int t = 0; t < tr.T; ++t) {
        if (obs.primed() && t >= 11) {
            const auto& h = obs.history();
            for (int i = 0; i < kNumHist; ++i) chh.add(h[i], tr.hist[t][i], i, 0.0f, 0.0f);
        }
        // 이 프레임의 prop 은 기록값을 그대로 쓴다 ([1] 에서 이미 검증했다).
        if (t == 0) obs.prime(tr.prop[t]); else obs.push(tr.prop[t]);
    }
    std::printf("[2] history 530 정렬      최대오차 %.3e  불일치 %d/%d  → %s\n",
                chh.worst_abs, chh.bad, chh.n, chh.bad ? "FAIL" : "PASS");
    if (chh.bad) ++fails;

    // -------------------------------------------------------------- [3] 액션 지연
    act.reset();
    Cmp ca;
    for (int t = 0; t < tr.T; ++t) {
        std::array<float, kNumJoints> a{};
        for (int i = 0; i < kNumJoints; ++i) a[i] = tr.actions[t][i];
        const auto q = act.push(a);
        if (t >= 1) {
            for (int i = 0; i < kNumJoints; ++i) {
                const float want = std::min(std::max(tr.actions[t - 1][i], c.action_clip_lo),
                                            c.action_clip_hi) * c.action_scale +
                                   c.default_joint_pos[i];
                ca.add(q[i], want, i);
            }
        }
        // 다음 관측에 들어갈 raw action 이 기록값과 같은가
        if (t + 1 < tr.T) {
            for (int i = 0; i < kNumJoints; ++i)
                ca.add(act.last_raw()[i], tr.prop[t + 1][37 + i], 100 + i);
        }
    }
    std::printf("[3] 액션 지연/스케일      최대오차 %.3e  불일치 %d/%d  → %s\n",
                ca.worst_abs, ca.bad, ca.n, ca.bad ? "FAIL" : "PASS");
    if (ca.bad) ++fails;

    // ---------------------------------------------------------------- [4] ONNX
    isaaclab::OrtRunner runner(argv[3]);
    Cmp co;
    for (int t = 0; t < tr.T; ++t) {
        std::unordered_map<std::string, std::vector<float>> in;
        in["prop"] = tr.prop[t];
        in["scan"] = tr.scan[t];
        in["hist"] = tr.hist[t];
        const auto out = runner.act(in);
        for (int i = 0; i < kNumJoints; ++i) co.add(out[i], tr.actions[t][i], i);
    }
    std::printf("[4] ONNX 정책 출력        최대오차 %.3e  불일치 %d/%d  → %s\n",
                co.worst_abs, co.bad, co.n, co.bad ? "FAIL" : "PASS");
    if (co.bad) {
        std::printf("    최악: idx %d  got %.6f  want %.6f\n",
                    co.worst_idx, co.worst_got, co.worst_want);
        ++fails;
    }

    std::printf("\n[RESULT] %s\n", fails ? "FAILED" : "OK");
    return fails ? 1 : 0;
}
