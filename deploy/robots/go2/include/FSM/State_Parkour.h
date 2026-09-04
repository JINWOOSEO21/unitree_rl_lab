// 파쿠르 정책 FSM 상태.
//
// State_RLBase 를 쓸 수 없는 이유: 그쪽은 isaaclab::ManagerBasedRLEnv 를 통해
// ObservationManager 로 관측을 만드는데, 거기 등록된 term 이 순수 proprioception
// 뿐이라 우리 obs 753(scan 132 + history 530 포함)을 표현할 수 없다. 미등록 term 은
// throw 한다. 그래서 관측 조립을 parkour/*.h 에서 직접 한다.
//
// 스레드 구조는 State_RLBase 와 같다:
//   FSM 스레드(1 kHz)  : run() — 최신 q_target 을 lowcmd 에 쓴다
//   정책 스레드(50 Hz) : 관측 조립 → ONNX → 액션 후처리
//
// scandots 132 는 별도 프로세스(EM 사이드카)가 rt/parkour/scandots 로 준다.
// 그것이 끊기면 정책은 낡은 지형을 보고 달리므로 Passive 로 떨어뜨린다.
#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <thread>

#include "FSM/FSMState.h"
#include "isaaclab/algorithms/algorithms.h"
#include "parkour/action.h"
#include "parkour/contract.h"
#include "parkour/heading.h"
#include "parkour/observation.h"
#include "parkour/scandots.h"

class State_Parkour : public FSMState
{
public:
    State_Parkour(int state_mode, std::string state_string);
    ~State_Parkour() { stop_thread(); }

    void enter() override;
    void run() override;
    void exit() override { stop_thread(); }

private:
    void policy_step();
    void stop_thread();
    /// 몸이 뒤집혔는가 (projected gravity 의 z 성분으로 판정, 임계는 rad).
    bool bad_orientation(float limit_rad) const;

    parkour::Contract contract_;
    std::unique_ptr<parkour::ObservationBuilder> obs_;
    std::unique_ptr<parkour::ActionPipeline> act_;
    std::unique_ptr<parkour::ScandotsSubscriber> scan_;
    std::unique_ptr<isaaclab::OrtRunner> policy_;
    parkour::HeadingCommand heading_;

    // 설정 (config.yaml FSM.Parkour)
    double scandots_timeout_s_ = 0.5;
    float contact_threshold_ = 2.0f;
    float cmd_vx_min_ = 0.3f;
    float cmd_vx_max_ = 0.8f;
    float bad_orientation_rad_ = 1.0f;

    // FSM 스레드가 읽는 값
    std::mutex q_mtx_;
    std::array<float, parkour::kNumJoints> q_target_{};
    std::atomic<bool> have_target_{false};
    std::atomic<bool> tripped_{false};   // 안전장치 로그를 한 번만 찍기 위한 래치

    // 접촉 필터용 직전 프레임 (학습은 now|prev 로 판정한다)
    std::array<bool, parkour::kNumFeet> prev_contact_{};

    std::thread thread_;
    std::atomic<bool> running_{false};
    uint64_t step_count_ = 0;
};

REGISTER_FSM(State_Parkour)
