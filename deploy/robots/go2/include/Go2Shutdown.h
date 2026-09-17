#pragma once

#include <array>
#include <chrono>
#include <functional>
#include "FSM/FSMState.h"
#include "ShutdownControl.h"

inline Go2ShutdownControl go2_shutdown;
inline std::function<uint64_t()> go2_shutdown_receipt_ns;
inline std::array<float, 12> go2_shutdown_down_q{}; // Set before starting FSM.

// Base FSMState installs operator DSL routes followed by its LowState timeout.
// Suppress operator routes during shutdown, preserving the timeout fail-safe.
inline void go2_guard_operator_routes(FSMState& state)
{
    for (size_t i = 0; i + 1 < state.registered_checks.size(); ++i) {
        const auto check = state.registered_checks[i].first;
        state.registered_checks[i].first = [check] { return !go2_shutdown.requested() && check(); };
    }
}

inline Go2ShutdownAction go2_shutdown_step(Go2RuntimeState state, bool stand_ready,
                                          bool motion_complete)
{
    const auto now = std::chrono::steady_clock::now().time_since_epoch();
    const uint64_t now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();
    bool valid = true, down = true, upright = false;
    uint32_t tick;
    {
        std::lock_guard<std::mutex> lock(FSMState::lowstate->mutex_);
        const auto& msg = FSMState::lowstate->msg_;
        tick = msg.tick();
        for (size_t i = 0; i < 12; ++i) {
            const auto& motor = msg.motor_state()[i];
            valid = valid && std::isfinite(motor.q()) && std::isfinite(motor.dq());
            // Same folded-pose envelope as startup; hip spread is allowed.
            const float tolerance = i % 3 == 0 ? 0.50f : 0.25f;
            down = down && go2_down_joint_settled(motor.q(), go2_shutdown_down_q[i],
                                                 motor.dq(), tolerance);
        }
        const auto& q = msg.imu_state().quaternion();
        Eigen::Quaternionf quat(q[0], q[1], q[2], q[3]);
        const float norm = quat.norm();
        valid = valid && std::isfinite(norm) && norm > 1e-6f;
        if (std::isfinite(norm) && norm > 1e-6f) {
            quat.normalize();
            const auto gravity = quat.conjugate() * Eigen::Vector3f(0, 0, -1);
            upright = std::acos(std::clamp(-gravity.z(), -1.0f, 1.0f)) <= 0.3f;
        }
    }
    const uint64_t receipt = go2_shutdown_receipt_ns ? go2_shutdown_receipt_ns() : 0;
    const bool fresh = receipt && receipt <= now_ns && now_ns - receipt <= 100'000'000;
    return go2_shutdown.step(state, tick, now_ns, fresh,
                             valid, upright, stand_ready, motion_complete, down);
}

// Append after safety checks, before any keyboard/operator routes.
inline void go2_add_shutdown_routes(FSMState& state, Go2RuntimeState runtime,
                                    std::function<bool()> stand_ready = [] { return false; },
                                    std::function<bool()> motion_complete = [] { return true; })
{
    auto action = std::make_shared<Go2ShutdownAction>(Go2ShutdownAction::Hold);
    state.registered_checks.emplace_back([runtime, stand_ready, motion_complete, action] {
        if (!go2_shutdown.requested()) return false;
        *action = go2_shutdown_step(runtime, stand_ready(), motion_complete());
        return *action == Go2ShutdownAction::Stand;
    }, FSMStringMap.right.at("FixStand"));
    state.registered_checks.emplace_back([action] {
        return go2_shutdown.requested() && *action == Go2ShutdownAction::Down;
    }, FSMStringMap.right.at("StandDown"));
    // A true same-state route blocks operator transitions while waiting.
    state.registered_checks.emplace_back([] { return go2_shutdown.requested(); }, state.getState());
}
