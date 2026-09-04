// deploy/parkour/contract/deploy.yaml 로더.
//
// 이 파일들(관절 순서 3종, 기본 자세, 액추에이터, 액션 스케일/지연, 타이밍)은
// 전부 IsaacLab 에서 **실측**해 못박은 값이다. C++ 쪽에 숫자를 옮겨 적지 않는다 —
// 옮겨 적는 순간 두 벌이 되고, 한쪽만 고치면 조용히 어긋난다.
#pragma once

#include <stdexcept>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

namespace parkour
{

struct Contract
{
    // index_maps
    std::vector<int> il_to_sdk;       // lowcmd.motor_cmd[il_to_sdk[i]] <- 정책 i 번
    std::vector<int> il_foot_to_sdk;  // lowstate.foot_force[il_foot_to_sdk[i]]

    // default_joint_pos.isaaclab — action offset 이자 joint_pos 관측의 기준
    std::vector<float> default_joint_pos;

    // actuator (isaaclab 순서). 배포는 q/kp/kd 만 보내므로 stiffness/damping 만 쓴다.
    std::vector<float> stiffness;
    std::vector<float> damping;

    // action
    float action_scale = 0.25f;
    float action_clip_lo = -4.8f;
    float action_clip_hi = 4.8f;
    int action_delay_steps = 1;

    // timing
    double step_dt = 0.02;   // 정책 50 Hz
    int em_tick_steps = 5;   // scandots / delta_yaw 는 10 Hz

    static Contract load(const std::string& yaml_path)
    {
        YAML::Node n = YAML::LoadFile(yaml_path);
        Contract c;

        // 주의: YAML::Node 는 **값(핸들)으로 다뤄야 한다.** `n["a"]` 는 임시를
        // 돌려주므로 `const YAML::Node& x = n["a"];` 로 받으면 그 문장이 끝나는
        // 순간 댕글링이 되고, 나중에 쓰면 세그폴트다 (실제로 그렇게 죽었다).
        auto req = [](YAML::Node node, const char* what) -> YAML::Node {
            if (!node) throw std::runtime_error(std::string("deploy.yaml: ") + what + " 가 없다");
            return node;
        };

        c.il_to_sdk = req(n["index_maps"]["il_to_sdk"], "index_maps.il_to_sdk")
                          .as<std::vector<int>>();
        c.il_foot_to_sdk = req(n["index_maps"]["il_foot_to_sdk"], "index_maps.il_foot_to_sdk")
                               .as<std::vector<int>>();
        c.default_joint_pos = req(n["default_joint_pos"]["isaaclab"], "default_joint_pos.isaaclab")
                                  .as<std::vector<float>>();
        c.stiffness = req(n["actuator"]["stiffness"], "actuator.stiffness").as<std::vector<float>>();
        c.damping = req(n["actuator"]["damping"], "actuator.damping").as<std::vector<float>>();

        YAML::Node act = req(n["action"], "action");
        c.action_scale = act["scale"].as<float>();
        auto clip = act["clip"].as<std::vector<float>>();
        if (clip.size() != 2) throw std::runtime_error("deploy.yaml: action.clip 은 2개여야 한다");
        c.action_clip_lo = clip[0];
        c.action_clip_hi = clip[1];
        c.action_delay_steps = act["delay_steps"].as<int>();

        YAML::Node tm = req(n["timing"], "timing");
        c.step_dt = tm["step_dt"].as<double>();
        c.em_tick_steps = tm["em_tick_steps"].as<int>();

        // 순서 계약이 어긋나면 여기서 죽는 편이 낫다. 조용히 틀린 관절로 나가는 것보다.
        if (c.il_to_sdk.size() != 12 || c.default_joint_pos.size() != 12 ||
            c.stiffness.size() != 12 || c.damping.size() != 12) {
            throw std::runtime_error("deploy.yaml: 12개짜리 배열의 길이가 맞지 않는다");
        }
        if (c.il_foot_to_sdk.size() != 4) {
            throw std::runtime_error("deploy.yaml: il_foot_to_sdk 는 4개여야 한다");
        }
        return c;
    }
};

}  // namespace parkour
