#include "FSM/State_Parkour.h"

#include <chrono>
#include <cmath>

#include "parkour/math.h"

using namespace parkour;

namespace
{
/// policy_dir 을 proj_dir 기준으로 푼다.
///
/// param::parser_policy_dir() 을 쓰지 않는 이유: 그 함수는 `exported` 폴더가 없으면
/// **하위 디렉터리를 훑어** 최신 것을 고르려 한다. 경로가 아예 없을 때
/// directory_iterator 가 던지는 filesystem_error 로 프로세스가 죽는데(실제로 Velocity
/// 상태가 그렇게 죽었다), 우리 계약 폴더는 구조가 평평해서 그 탐색이 아무 도움도 안 된다.
inline std::filesystem::path resolve_policy_dir(const std::string& state_string)
{
    std::filesystem::path d =
        param::config["FSM"][state_string]["policy_dir"].as<std::string>();
    if (d.is_relative()) d = param::proj_dir / d;
    if (!std::filesystem::exists(d)) {
        throw std::runtime_error("State_Parkour: policy_dir 이 없다: " + d.string());
    }
    return std::filesystem::weakly_canonical(d);
}

/// config 에서 값을 읽되, 없으면 기본값을 쓴다 (설정을 필수로 만들지 않는다).
template <typename T>
T cfg_or(const YAML::Node& n, const char* key, T fallback)
{
    return n[key] ? n[key].as<T>() : fallback;
}
}  // namespace

State_Parkour::State_Parkour(int state_mode, std::string state_string)
    : FSMState(state_mode, state_string),
      contract_(Contract::load(resolve_policy_dir(state_string) / "deploy.yaml"))
{
    auto cfg = param::config["FSM"][state_string];
    const auto dir = resolve_policy_dir(state_string);

    scandots_timeout_s_ = cfg_or<double>(cfg, "scandots_timeout", 0.5);
    // 실기의 foot_force 는 정수 원시값이라 스케일이 N 이 아니다. MuJoCo 브리지는
    // touch 센서의 물리량(N)을 그대로 싣는다 — 학습 임계 2 N 이 그대로 맞는다.
    // 실기로 옮길 때 이 값을 반드시 다시 잡을 것 [실측 필요].
    contact_threshold_ = cfg_or<float>(cfg, "contact_threshold", 2.0f);
    // 학습 명령 분포는 0.3~0.8 m/s 다. 0 은 분포 밖이라 넣지 않는다 —
    // 멈추려면 Passive(LT+B)로 나간다.
    cmd_vx_min_ = cfg_or<float>(cfg, "cmd_vx_min", 0.3f);
    cmd_vx_max_ = cfg_or<float>(cfg, "cmd_vx_max", 0.8f);
    bad_orientation_rad_ = cfg_or<float>(cfg, "bad_orientation", 1.0f);
    log_path_ = cfg_or<std::string>(cfg, "log_obs", std::string(""));

    // 실험용 덮어쓰기(-1 이면 계약값 그대로). 학습은 액션을 1 스텝(20 ms) 늦춰
    // 적용하고 정책은 그 지연을 전제로 학습됐다. 그런데 배포에는 DDS 왕복과 시뮬
    // 적용까지 **실제 지연**이 이미 있다. 그 위에 1 스텝을 더 얹으면 총 지연이
    // 학습보다 커진다. 어느 쪽이 학습에 가까운지는 재 봐야 안다.
    const int dly = cfg_or<int>(cfg, "action_delay_override", -1);
    if (dly >= 0 && dly != contract_.action_delay_steps) {
        spdlog::warn("State_Parkour: action_delay_steps {} → {} 로 덮어씀 (실험용)",
                     contract_.action_delay_steps, dly);
        contract_.action_delay_steps = dly;
    }

    obs_ = std::make_unique<ObservationBuilder>(contract_);
    act_ = std::make_unique<ActionPipeline>(contract_);
    scan_ = std::make_unique<ScandotsSubscriber>(
        cfg_or<std::string>(cfg, "scandots_topic", "rt/parkour/scandots"));
    policy_ = std::make_unique<isaaclab::OrtRunner>((dir / "policy.onnx").string());

    // 넘어지면 Passive.
    // **어느 안전장치가 걸렸는지 반드시 남긴다.** 둘 다 "Parkour → Passive" 로만 보이면
    // 넘어진 것인지 지각이 끊긴 것인지 구분할 수 없고, 그 둘은 대응이 완전히 다르다.
    registered_checks.emplace_back(
        [this]() -> bool {
            if (!this->bad_orientation(bad_orientation_rad_)) return false;
            if (!this->tripped_.exchange(true))
                spdlog::warn("State_Parkour: 자세 이탈 (기울기 > {} rad) → Passive",
                             bad_orientation_rad_);
            return true;
        },
        FSMStringMap.right.at("Passive"));

    // scandots 가 끊기면 Passive. 이게 없으면 사이드카가 죽어도 정책은 낡은 지형을
    // 보고 계속 달린다 — 조용히 위험해지는 종류라 명시적으로 막는다.
    registered_checks.emplace_back(
        [this]() -> bool {
            if (!this->running_.load()) return false;
            const double age = this->scan_->last_age();
            if (age <= this->scandots_timeout_s_) return false;
            if (!this->tripped_.exchange(true))
                spdlog::warn("State_Parkour: scandots 끊김 ({:.2f}s > {:.2f}s, 누적 {}건) → Passive",
                             age, this->scandots_timeout_s_, this->scan_->count());
            return true;
        },
        FSMStringMap.right.at("Passive"));

    spdlog::info("State_Parkour: policy={} scandots_timeout={}s contact_thr={}",
                 (dir / "policy.onnx").string(), scandots_timeout_s_, contact_threshold_);
}

bool State_Parkour::bad_orientation(float limit_rad) const
{
    const auto& q = lowstate->msg_.imu_state().quaternion();
    Eigen::Quaternionf quat(q[0], q[1], q[2], q[3]);
    // 몸 기준에서 본 중력. 똑바로 서 있으면 (0, 0, -1).
    const Eigen::Vector3f g_b = quat.conjugate() * Eigen::Vector3f(0.0f, 0.0f, -1.0f);
    const float tilt = std::acos(std::clamp(-g_b.z(), -1.0f, 1.0f));
    return tilt > limit_rad;
}

void State_Parkour::enter()
{
    // PD 게인은 계약(학습값 kp=40, kd=1)을 쓴다.
    for (int i = 0; i < kNumJoints; ++i) {
        const int sdk = contract_.il_to_sdk[i];
        auto& m = lowcmd->msg_.motor_cmd()[sdk];
        m.kp() = contract_.stiffness[i];
        m.kd() = contract_.damping[i];
        m.dq() = 0;
        m.tau() = 0;
    }

    obs_->reset();
    act_->reset();
    have_target_ = false;
    tripped_ = false;
    step_count_ = 0;
    prev_contact_.fill(false);

    {
        const auto& q = lowstate->msg_.imu_state().quaternion();
        heading_.reset(yaw_from_quat(Eigen::Quaternionf(q[0], q[1], q[2], q[3])));
    }

    if (scan_->count() == 0) {
        spdlog::warn("State_Parkour: scandots 를 아직 한 번도 못 받았다 "
                     "(EM 사이드카가 떠 있는가?). 평지로 가정하고 시작한다.");
    }

    if (!log_path_.empty()) {
        log_ = std::fopen(log_path_.c_str(), "wb");
        if (log_) {
            std::fwrite("PKOB", 1, 4, log_);          // magic
            // t, prop, scan, action, scan_age, scan_count  (진단 2개 포함)
            const int rec = 1 + kNumProp + kNumScan + kNumJoints + 2;   // 200
            std::fwrite(&rec, sizeof(int), 1, log_);
            log_t0_ = std::chrono::duration<double>(
                std::chrono::steady_clock::now().time_since_epoch()).count();
            spdlog::info("State_Parkour: 관측 녹화 → {}", log_path_);
        } else {
            spdlog::warn("State_Parkour: 로그 파일을 열 수 없다: {}", log_path_);
        }
    }

    running_ = true;
    thread_ = std::thread([this] {
        using clock = std::chrono::high_resolution_clock;
        const auto dt = std::chrono::duration_cast<clock::duration>(
            std::chrono::duration<double>(contract_.step_dt));
        auto next = clock::now() + dt;
        while (running_.load()) {
            policy_step();
            std::this_thread::sleep_until(next);
            next += dt;
        }
    });
}

void State_Parkour::policy_step()
{
    PropInputs in;

    // --- lowstate 읽기 (SDK 순서 → IsaacLab 순서) ---
    {
        std::lock_guard<std::mutex> lk(lowstate->mutex_);
        const auto& imu = lowstate->msg_.imu_state();
        in.gyro_b = Eigen::Vector3f(imu.gyroscope()[0], imu.gyroscope()[1], imu.gyroscope()[2]);
        in.quat_w = Eigen::Quaternionf(imu.quaternion()[0], imu.quaternion()[1],
                                       imu.quaternion()[2], imu.quaternion()[3]);
        for (int i = 0; i < kNumJoints; ++i) {
            const int sdk = contract_.il_to_sdk[i];
            in.joint_pos[i] = lowstate->msg_.motor_state()[sdk].q();
            in.joint_vel[i] = lowstate->msg_.motor_state()[sdk].dq();
        }
        for (int k = 0; k < kNumFeet; ++k) {
            const float f = lowstate->msg_.foot_force()[contract_.il_foot_to_sdk[k]];
            const bool now = f > contact_threshold_;
            // 학습과 같은 규약: 이번 프레임 **또는** 직전 프레임에 접촉이면 1.
            in.contact[k] = now || prev_contact_[k];
            prev_contact_[k] = now;
        }
    }

    // --- 조향/속도 명령 (조이스틱) ---
    // delta_yaw 는 학습에서 10 Hz 로만 갱신됐다. 같은 위상으로 맞춘다.
    const float yaw_now = yaw_from_quat(in.quat_w);
    auto* joy = &lowstate->joystick;
    if (step_count_ % static_cast<uint64_t>(contract_.em_tick_steps) == 0) {
        heading_.update(joy->rx(), yaw_now, contract_.step_dt * contract_.em_tick_steps);
    } else {
        heading_.set_yaw(yaw_now);
    }
    in.delta_yaw = heading_.delta_yaw();
    in.delta_next_yaw = heading_.delta_next_yaw();
    const float ly = std::clamp(joy->ly(), 0.0f, 1.0f);
    in.cmd_vx = cmd_vx_min_ + ly * (cmd_vx_max_ - cmd_vx_min_);

    for (int i = 0; i < kNumJoints; ++i) in.last_action[i] = act_->last_raw()[i];

    // --- 관측 조립 ---
    const auto prop = obs_->build_prop(in);
    if (!obs_->primed()) obs_->prime(prop);

    std::unordered_map<std::string, std::vector<float>> feed;
    feed["prop"] = prop;
    feed["scan"] = scan_->get();
    feed["hist"] = obs_->history();

    // --- 정책 ---
    const auto raw = policy_->act(feed);
    std::array<float, kNumJoints> a{};
    for (int i = 0; i < kNumJoints; ++i) a[i] = raw[i];

    // --- 액션 후처리 (1스텝 지연 → 클립 → 스케일 → 기본자세) ---
    const auto q = act_->push(a);
    {
        std::lock_guard<std::mutex> lk(q_mtx_);
        q_target_ = q;
        target_stamp_us_ = now_us();     // 지연 계측용 (run() 이 lowcmd 에 실어 보낸다)
    }
    have_target_ = true;

    // 관측을 만든 **뒤에** history 에 넣는다 (학습과 같은 순서 — hist 는 현재 프레임을
    // 포함하지 않는다). prime() 을 이미 했으므로 여기서는 밀어 넣기만 한다.
    if (log_) {
        const float t = static_cast<float>(
            std::chrono::duration<double>(
                std::chrono::steady_clock::now().time_since_epoch()).count() - log_t0_);
        std::fwrite(&t, sizeof(float), 1, log_);
        std::fwrite(prop.data(), sizeof(float), kNumProp, log_);
        std::fwrite(feed["scan"].data(), sizeof(float), kNumScan, log_);
        std::fwrite(a.data(), sizeof(float), kNumJoints, log_);
        // scandots 신선도. 0.5s 가드에는 안 걸려도 지도가 갱신을 멈춘 채 유효한 값을
        // 계속 내보내면 정책 입력이 얼어붙는다 — 그 경우 age 는 작은데 count 가
        // 멈춘다. 값 자체의 정지 여부는 기록된 scan 을 프레임 간 비교해 본다.
        const float age = static_cast<float>(scan_->last_age());
        const float cnt = static_cast<float>(scan_->count());
        std::fwrite(&age, sizeof(float), 1, log_);
        std::fwrite(&cnt, sizeof(float), 1, log_);
    }

    obs_->push(prop);
    ++step_count_;
}

void State_Parkour::run()
{
    if (!have_target_.load()) return;  // 첫 정책 스텝 전에는 손대지 않는다
    std::array<float, kNumJoints> q;
    uint32_t stamp;
    {
        std::lock_guard<std::mutex> lk(q_mtx_);
        q = q_target_;
        stamp = target_stamp_us_;
    }
    for (int i = 0; i < kNumJoints; ++i) {
        lowcmd->msg_.motor_cmd()[contract_.il_to_sdk[i]].q() = q[i];
    }
    // 지연 계측용: 이 목표각을 **정책이 만든 시각**을 실어 보낸다. 시뮬레이터
    // 브리지가 토크를 거는 순간 이 값의 나이를 재면 종단간 지연이 그대로 나온다.
    // (같은 머신이라 CLOCK_MONOTONIC 이 두 프로세스에서 공유된다.)
    // 실기에서는 이 필드를 읽는 쪽이 없으므로 무해하다.
    lowcmd->msg_.reserve() = stamp;
}

uint32_t State_Parkour::now_us()
{
    return static_cast<uint32_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count());
}

void State_Parkour::stop_thread()
{
    running_ = false;
    if (thread_.joinable()) thread_.join();
    if (log_) {
        std::fclose(log_);
        log_ = nullptr;
        spdlog::info("State_Parkour: 관측 녹화 종료 ({} 스텝)", step_count_);
    }
}
