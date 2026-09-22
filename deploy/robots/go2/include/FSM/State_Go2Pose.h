#pragma once

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <limits>

#include "FSM/FSMState.h"
#include "KeyboardControl.h"
#include "Go2Shutdown.h"
#include "GyroBiasSubscriber.h"
#include "parkour/scandots.h"

class State_Go2Pose : public FSMState
{
public:
    State_Go2Pose(int state, std::string state_string)
        : FSMState(state, state_string)
    {
        go2_guard_operator_routes(*this);
        const auto cfg = param::config["FSM"]["FixStand"];
        kp_ = cfg["kp"].as<std::vector<float>>();
        kd_ = cfg["kd"].as<std::vector<float>>();
        const auto poses = cfg["qs"].as<std::vector<std::vector<float>>>();
        down_ = state_string == "StandDown";
        target_ = down_ ? poses.at(1) : poses.back();
        duration_s_ = cfg["pose_duration"] ? cfg["pose_duration"].as<double>() : 2.0;
        if (down_) duration_s_ = cfg["standdown_duration"] ? cfg["standdown_duration"].as<double>() : 3.0;
        stand_tolerance_ = cfg["stand_tolerance"] ? cfg["stand_tolerance"].as<float>() : 0.2f;
        standdown_tolerance_ = cfg["standdown_tolerance"] ? cfg["standdown_tolerance"].as<float>() : 0.15f;
        if (!std::isfinite(standdown_tolerance_) || standdown_tolerance_ <= 0)
            throw std::runtime_error("Go2Pose standdown_tolerance must be finite and positive");
        if (target_.size() != 12 || kp_.size() != 12 || kd_.size() != 12)
            throw std::runtime_error("Go2Pose gain/target size mismatch");
        if (!std::isfinite(duration_s_) || duration_s_ <= 0 ||
            !std::isfinite(stand_tolerance_) || stand_tolerance_ < 0)
            throw std::runtime_error("Go2Pose duration/tolerance is invalid");
        for (size_t i = 0; i < target_.size(); ++i)
            if (!std::isfinite(target_[i]) || !std::isfinite(kp_[i]) || kp_[i] < 0 ||
                !std::isfinite(kd_[i]) || kd_[i] < 0)
                throw std::runtime_error("Go2Pose target/gain contains an invalid value");
        {
            const auto parkour_cfg = param::config["FSM"]["Parkour"];
            scan_timeout_s_ = parkour_cfg["scandots_timeout"].as<double>();
            bad_orientation_rad_ = parkour_cfg["bad_orientation"].as<float>();
            scan_ = std::make_unique<parkour::ScandotsSubscriber>(
                parkour_cfg["scandots_topic"].as<std::string>());
        }
        go2_add_shutdown_routes(*this, down_ ? Go2RuntimeState::StandDown : Go2RuntimeState::Stand,
            [this] { return complete_.load() && down_gate_.ready() && upright(0.3f); },
            [this] { return complete_.load(); });
        add_routes();
    }

    void enter() override
    {
        if (go2_keyboard_control)
            go2_keyboard_control->set_state(down_ ? Go2RuntimeState::StandDown : Go2RuntimeState::Stand);
        complete_ = false;
        down_gate_.reset();
        start_.resize(target_.size());
        {
            std::lock_guard<std::mutex> lock(lowstate->mutex_);
            for (size_t i = 0; i < target_.size(); ++i) start_[i] = lowstate->msg_.motor_state()[i].q();
        }
        if (!go2_pose_within_tolerance(start_, start_, 0.0f))
            throw std::runtime_error("Go2Pose measured starting joints are non-finite");
        for (size_t i = 0; i < target_.size(); ++i) {
            auto& motor = lowcmd->msg_.motor_cmd()[i];
            motor.kp() = kp_[i];
            motor.kd() = kd_[i];
            motor.dq() = 0;
            motor.tau() = 0;
            motor.q() = start_[i];
        }
        began_ = Clock::now();
    }

    void run() override
    {
        const double elapsed = std::chrono::duration<double>(Clock::now() - began_).count();
        for (size_t i = 0; i < target_.size(); ++i)
            lowcmd->msg_.motor_cmd()[i].q() = down_
                ? go2_pose_smooth(start_[i], target_[i], elapsed, duration_s_)
                : go2_pose_lerp(start_[i], target_[i], elapsed, duration_s_);
        if (elapsed >= duration_s_) complete_ = true;
        if (!down_) {
            bool settled = complete_.load() && upright(0.3f);
            std::lock_guard<std::mutex> lock(lowstate->mutex_);
            for (size_t i = 0; i < target_.size(); ++i) {
                const auto& motor = lowstate->msg_.motor_state()[i];
                settled = settled && go2_down_joint_settled(
                    motor.q(), target_[i], motor.dq(), standdown_tolerance_);
            }
            down_gate_.update(lowstate->msg_.tick(), settled);
        }
    }

private:
    using Clock = std::chrono::steady_clock;

    float tilt_radians() const
    {
        std::lock_guard<std::mutex> lock(lowstate->mutex_);
        const auto& q = lowstate->msg_.imu_state().quaternion();
        if (!std::isfinite(q[0]) || !std::isfinite(q[1]) || !std::isfinite(q[2]) || !std::isfinite(q[3]))
            return std::numeric_limits<float>::quiet_NaN();
        Eigen::Quaternionf quat(q[0], q[1], q[2], q[3]);
        const float norm = quat.norm();
        if (!std::isfinite(norm) || norm < 1e-6f) return std::numeric_limits<float>::quiet_NaN();
        quat.normalize();
        const Eigen::Vector3f gravity = quat.conjugate() * Eigen::Vector3f(0, 0, -1);
        const float tilt = std::acos(std::clamp(-gravity.z(), -1.0f, 1.0f));
        return tilt;
    }

    bool upright(float limit) const
    {
        const float tilt = tilt_radians();
        return std::isfinite(tilt) && tilt <= limit;
    }

    void log_down_rejection() const
    {
        const float tilt = tilt_radians();
        std::lock_guard<std::mutex> lock(lowstate->mutex_);
        size_t worst = 0, fastest = 0;
        float error = -1, speed = -1;
        bool finite = true;
        for (size_t i = 0; i < target_.size(); ++i) {
            const auto& motor = lowstate->msg_.motor_state()[i];
            finite = finite && std::isfinite(motor.q()) && std::isfinite(motor.dq());
            const float e = std::abs(motor.q()-target_[i]);
            const float v = std::abs(motor.dq());
            if (e > error) { error = e; worst = i; }
            if (v > speed) { speed = v; fastest = i; }
        }
        spdlog::warn("StandDown rejected: complete={} dwell_500ms={} joints_finite={}; "
                     "SDK joint[{}] q={:.4f} target={:.4f} error={:.4f}/{:.4f} rad; "
                     "max |dq| joint[{}]={:.4f}/0.2000 rad/s; tilt={:.4f}/0.3000 rad. "
                     "Wait for settled Stand, then press 3 again",
                     complete_.load(), down_gate_.ready(), finite, worst,
                     lowstate->msg_.motor_state()[worst].q(), target_[worst], error,
                     standdown_tolerance_, fastest, speed, tilt);
    }

    // Policy 진입 거부 이유. 키보드('2')와 조종기(Start) 경로가 같이 쓴다.
    void log_policy_rejection(const Go2PolicyReadiness& ready) const
    {
        if (!ready.stand_complete) {
            size_t worst = 0;
            float error = -1, q_worst = 0;
            {
                std::lock_guard<std::mutex> lock(lowstate->mutex_);
                for (size_t i = 0; i < target_.size(); ++i) {
                    const float e = std::abs(lowstate->msg_.motor_state()[i].q() - target_[i]);
                    if (e > error) { error = e; worst = i; q_worst = lowstate->msg_.motor_state()[i].q(); }
                }
            }
            spdlog::warn("Policy entry rejected: stand motion incomplete or joint error exceeds tolerance "
                         "(motion_complete={}, worst SDK joint[{}] q={:.3f} target={:.3f} error={:.3f}/{:.3f} rad)",
                         complete_.load(), worst, q_worst, target_[worst], error, stand_tolerance_);
        }
        else if (!ready.scan_fresh_and_valid)
            spdlog::warn("Policy entry rejected: scandots are missing, stale, non-finite, or out of range");
        else if (!ready.gyro_bias_valid) {
            parkour::GyroBiasSample bias;
            std::string reason = "gyro bias subscriber unavailable";
            if (go2_gyro_bias) go2_gyro_bias->get(bias, &reason);
            spdlog::warn("Policy entry rejected: {}", reason);
        } else
            spdlog::warn("Policy entry rejected: IMU orientation is invalid or not upright");
    }

    Go2PolicyReadiness readiness() const
    {
        Go2PolicyReadiness result;
        parkour::GyroBiasSample bias;
        result.gyro_bias_valid = go2_gyro_bias && go2_gyro_bias->get(bias);
        std::vector<float> measured(target_.size());
        {
            std::lock_guard<std::mutex> lock(lowstate->mutex_);
            for (size_t i = 0; i < target_.size(); ++i) measured[i] = lowstate->msg_.motor_state()[i].q();
        }
        result.stand_complete = complete_.load() &&
                                go2_pose_within_tolerance(measured, target_, stand_tolerance_);
        result.upright = upright(bad_orientation_rad_);
        if (!scan_) return result;
        std::vector<float> values;
        if (!scan_->get_if_fresh(scan_timeout_s_, values)) return result;
        result.scan_fresh_and_valid = go2_scan_values_valid(values);
        return result;
    }

    void add_routes()
    {
        auto check = [this](Go2RuntimeState target) {
            return [this, target] {
                if (!go2_keyboard_control) return false;
                Go2PolicyReadiness ready;
                const bool policy_request = go2_keyboard_control->pending_request() == Go2StateRequest::Policy;
                if (policy_request || go2_keyboard_control->pending_request() == Go2StateRequest::StandDown)
                    ready = readiness();
                if (go2_keyboard_control->pending_request() == Go2StateRequest::StandDown) {
                    ready.stand_complete = complete_.load() && down_gate_.ready();
                    ready.upright = upright(0.3f);
                    if (target == Go2RuntimeState::Passive && !down_ &&
                        !(ready.stand_complete && ready.upright)) {
                        if (go2_keyboard_control->consume_request(Go2StateRequest::StandDown))
                            log_down_rejection();
                        return false;
                    }
                }
                if (policy_request) {
                    if (target == Go2RuntimeState::Passive &&
                        !(ready.stand_complete && ready.scan_fresh_and_valid && ready.upright && ready.gyro_bias_valid)) {
                        if (go2_keyboard_control->consume_request(Go2StateRequest::Policy))
                            log_policy_rejection(ready);
                        return false;
                    }
                }
                return go2_keyboard_control->consume_transition(target, ready);
            };
        };
        registered_checks.emplace_back(check(Go2RuntimeState::Passive), FSMStringMap.right.at("Passive"));
        registered_checks.emplace_back(check(Go2RuntimeState::StandDown), FSMStringMap.right.at("StandDown"));
        registered_checks.emplace_back(check(Go2RuntimeState::Stand), FSMStringMap.right.at("FixStand"));
        registered_checks.emplace_back(check(Go2RuntimeState::Policy), FSMStringMap.right.at("Parkour"));
        {
            if (down_) return;  // Down must go through Stand before Policy, including joystick.
            unitree::common::dsl::Parser parser("start.on_pressed");
            auto ast = parser.Parse();
            auto joystick_policy = unitree::common::dsl::Compile(*ast);
            registered_checks.emplace_back(
                [this, joystick_policy] {
                    // Start 는 edge(on_pressed)라 매 tick 평가해야 누름을 놓치지 않는다. 조건이
                    // 안 맞으면 조용히 버리지 않고 이유를 남긴다 (누름당 한 번).
                    if (!joystick_policy(lowstate->joystick)) return false;
                    const auto ready = readiness();
                    if (ready.stand_complete && ready.scan_fresh_and_valid && ready.upright && ready.gyro_bias_valid)
                        return true;
                    log_policy_rejection(ready);
                    return false;
                },
                FSMStringMap.right.at("Parkour"));
        }
    }

    Go2StandDownGate down_gate_;
    bool down_ = false;
    double duration_s_ = 2.0;
    double scan_timeout_s_ = 0.5;
    float bad_orientation_rad_ = 1.0f;
    float stand_tolerance_ = 0.2f;
    float standdown_tolerance_ = 0.15f;
    std::vector<float> kp_, kd_, start_, target_;
    Clock::time_point began_{};
    std::atomic<bool> complete_{false};
    std::unique_ptr<parkour::ScandotsSubscriber> scan_;
};

REGISTER_FSM(State_Go2Pose)
