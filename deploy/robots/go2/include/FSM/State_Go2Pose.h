#pragma once

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>

#include "FSM/FSMState.h"
#include "KeyboardControl.h"
#include "parkour/scandots.h"

class State_Go2Pose : public FSMState
{
public:
    State_Go2Pose(int state, std::string state_string)
        : FSMState(state, state_string)
    {
        const auto cfg = param::config["FSM"]["FixStand"];
        kp_ = cfg["kp"].as<std::vector<float>>();
        kd_ = cfg["kd"].as<std::vector<float>>();
        const auto poses = cfg["qs"].as<std::vector<std::vector<float>>>();
        target_ = poses.back();
        duration_s_ = cfg["pose_duration"] ? cfg["pose_duration"].as<double>() : 2.0;
        stand_tolerance_ = cfg["stand_tolerance"] ? cfg["stand_tolerance"].as<float>() : 0.2f;
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
        add_routes();
    }

    void enter() override
    {
        if (go2_keyboard_control)
            go2_keyboard_control->set_state(Go2RuntimeState::Stand);
        complete_ = false;
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
            lowcmd->msg_.motor_cmd()[i].q() = go2_pose_lerp(start_[i], target_[i], elapsed, duration_s_);
        if (elapsed >= duration_s_) complete_ = true;
    }

private:
    using Clock = std::chrono::steady_clock;

    bool upright() const
    {
        std::lock_guard<std::mutex> lock(lowstate->mutex_);
        const auto& q = lowstate->msg_.imu_state().quaternion();
        if (!std::isfinite(q[0]) || !std::isfinite(q[1]) || !std::isfinite(q[2]) || !std::isfinite(q[3]))
            return false;
        Eigen::Quaternionf quat(q[0], q[1], q[2], q[3]);
        const float norm = quat.norm();
        if (!std::isfinite(norm) || norm < 1e-6f) return false;
        quat.normalize();
        const Eigen::Vector3f gravity = quat.conjugate() * Eigen::Vector3f(0, 0, -1);
        const float tilt = std::acos(std::clamp(-gravity.z(), -1.0f, 1.0f));
        return std::isfinite(tilt) && tilt <= bad_orientation_rad_;
    }

    Go2PolicyReadiness readiness() const
    {
        Go2PolicyReadiness result;
        std::vector<float> measured(target_.size());
        {
            std::lock_guard<std::mutex> lock(lowstate->mutex_);
            for (size_t i = 0; i < target_.size(); ++i) measured[i] = lowstate->msg_.motor_state()[i].q();
        }
        result.stand_complete = complete_.load() &&
                                go2_pose_within_tolerance(measured, target_, stand_tolerance_);
        result.upright = upright();
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
                if (policy_request) {
                    ready = readiness();
                    if (target == Go2RuntimeState::Passive &&
                        !(ready.stand_complete && ready.scan_fresh_and_valid && ready.upright)) {
                        if (go2_keyboard_control->consume_request(Go2StateRequest::Policy)) {
                            if (!ready.stand_complete)
                                spdlog::warn("Policy entry rejected: stand motion incomplete or joint error exceeds tolerance");
                            else if (!ready.scan_fresh_and_valid)
                                spdlog::warn("Policy entry rejected: scandots are missing, stale, non-finite, or out of range");
                            else
                                spdlog::warn("Policy entry rejected: IMU orientation is invalid or not upright");
                        }
                        return false;
                    }
                }
                return go2_keyboard_control->consume_transition(target, ready);
            };
        };
        registered_checks.emplace_back(check(Go2RuntimeState::Passive), FSMStringMap.right.at("Passive"));
        registered_checks.emplace_back(check(Go2RuntimeState::Stand), FSMStringMap.right.at("FixStand"));
        registered_checks.emplace_back(check(Go2RuntimeState::Policy), FSMStringMap.right.at("Parkour"));
        {
            unitree::common::dsl::Parser parser("start.on_pressed");
            auto ast = parser.Parse();
            auto joystick_policy = unitree::common::dsl::Compile(*ast);
            registered_checks.emplace_back(
                [this, joystick_policy] {
                    const auto ready = readiness();
                    return ready.stand_complete && ready.scan_fresh_and_valid && ready.upright &&
                           joystick_policy(lowstate->joystick);
                },
                FSMStringMap.right.at("Parkour"));
        }
    }

    double duration_s_ = 2.0;
    double scan_timeout_s_ = 0.5;
    float bad_orientation_rad_ = 1.0f;
    float stand_tolerance_ = 0.2f;
    std::vector<float> kp_, kd_, start_, target_;
    Clock::time_point began_{};
    std::atomic<bool> complete_{false};
    std::unique_ptr<parkour::ScandotsSubscriber> scan_;
};

REGISTER_FSM(State_Go2Pose)
