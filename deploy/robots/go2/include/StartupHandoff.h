#pragma once

#include <array>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <string>
#include <utility>

struct Go2StartupSample
{
    uint64_t receipt_ns = 0;
    uint32_t tick = 0;
    std::array<float, 12> q{};
    std::array<float, 12> dq{};
    std::array<float, 4> quaternion{};
};

struct Go2StartupHandoffConfig
{
    std::array<float, 12> down_q{};
    std::array<float, 12> joint_tolerance{};
    float velocity_tolerance = 0.20f;
    float quaternion_norm_tolerance = 0.10f;
    float max_body_tilt_rad = 0.60f;
    uint64_t lowstate_max_age_ns = 100'000'000;
    uint64_t down_stable_duration_ns = 500'000'000;
    int down_max_polls = 750;
    int release_max_polls = 60;
    int quiet_stable_samples = 20;
    int quiet_max_polls = 250;
};

struct Go2StartupHandoffHooks
{
    std::function<int(std::string&, std::string&)> check_mode;
    std::function<int()> stand_down;
    std::function<int()> release_mode;
    std::function<Go2StartupSample()> sample_lowstate;
    std::function<bool()> competing_lowcmd_active;
    std::function<bool()> cancelled;
    std::function<uint64_t()> now_ns;
    std::function<void()> wait_poll;
};

struct Go2StartupHandoffResult
{
    bool ready = false;
    std::string error;
};

inline bool go2_startup_sample_is_down(
    const Go2StartupSample& sample, const Go2StartupHandoffConfig& config)
{
    float quaternion_norm_sq = 0.0f;
    for (float value : sample.quaternion) {
        if (!std::isfinite(value)) return false;
        quaternion_norm_sq += value * value;
    }
    const float quaternion_norm = std::sqrt(quaternion_norm_sq);
    if (!std::isfinite(quaternion_norm) ||
        std::abs(quaternion_norm - 1.0f) > config.quaternion_norm_tolerance)
        return false;
    const float w = sample.quaternion[0] / quaternion_norm;
    const float x = sample.quaternion[1] / quaternion_norm;
    const float y = sample.quaternion[2] / quaternion_norm;
    const float z_alignment = 1.0f - 2.0f * (x * x + y * y);
    if (!std::isfinite(z_alignment) ||
        z_alignment < std::cos(config.max_body_tilt_rad))
        return false;

    for (size_t i = 0; i < config.down_q.size(); ++i) {
        if (!std::isfinite(sample.q[i]) || !std::isfinite(sample.dq[i]) ||
            !std::isfinite(config.joint_tolerance[i]) || config.joint_tolerance[i] < 0.0f ||
            std::abs(sample.q[i] - config.down_q[i]) > config.joint_tolerance[i] ||
            std::abs(sample.dq[i]) > config.velocity_tolerance)
            return false;
    }
    return true;
}

inline Go2StartupHandoffResult go2_perform_startup_handoff(
    const Go2StartupHandoffConfig& config, const Go2StartupHandoffHooks& hooks)
{
    auto fail = [](std::string error) {
        return Go2StartupHandoffResult{false, std::move(error)};
    };
    auto cancelled = [&hooks] { return hooks.cancelled && hooks.cancelled(); };
    auto wait = [&hooks] {
        if (hooks.wait_poll) hooks.wait_poll();
    };

    if (!hooks.check_mode || !hooks.sample_lowstate || !hooks.competing_lowcmd_active || !hooks.now_ns)
        return fail("startup handoff hooks are incomplete");
    if (config.down_stable_duration_ns == 0 || config.lowstate_max_age_ns == 0 ||
        config.down_max_polls <= 0 ||
        config.release_max_polls <= 0 || config.quiet_stable_samples <= 0 ||
        config.quiet_max_polls <= 0)
        return fail("startup handoff limits are invalid");
    if (!std::isfinite(config.velocity_tolerance) || config.velocity_tolerance < 0.0f ||
        !std::isfinite(config.quaternion_norm_tolerance) || config.quaternion_norm_tolerance < 0.0f ||
        !std::isfinite(config.max_body_tilt_rad) || config.max_body_tilt_rad < 0.0f ||
        config.max_body_tilt_rad > 3.1415927f)
        return fail("startup handoff tolerances are invalid");
    for (size_t i = 0; i < config.down_q.size(); ++i) {
        if (!std::isfinite(config.down_q[i]) || !std::isfinite(config.joint_tolerance[i]) ||
            config.joint_tolerance[i] < 0.0f)
            return fail("startup handoff joint target or tolerance is invalid");
    }

    auto sample_is_fresh_and_down = [&]() {
        const auto sample = hooks.sample_lowstate();
        const uint64_t now = hooks.now_ns();
        return sample.receipt_ns != 0 && sample.receipt_ns <= now &&
               now - sample.receipt_ns <= config.lowstate_max_age_ns &&
               go2_startup_sample_is_down(sample, config);
    };

    if (cancelled()) return fail("startup handoff cancelled before CheckMode");
    std::string form;
    std::string mode;
    const int initial_mode_result = hooks.check_mode(form, mode);
    if (initial_mode_result != 0)
        return fail("CheckMode failed before startup handoff: " + std::to_string(initial_mode_result));
    if (cancelled()) return fail("startup handoff cancelled after CheckMode");

    const bool sport_was_active = !mode.empty();
    const std::string initial_form = form;
    const std::string initial_mode = mode;
    if (sport_was_active) {
        if (!hooks.stand_down) return fail("StandDown hook is missing");
        const int stand_down_result = hooks.stand_down();
        if (stand_down_result != 0)
            return fail("StandDown failed: " + std::to_string(stand_down_result));
        if (cancelled()) return fail("startup handoff cancelled after StandDown");
    }

    bool have_tick = false;
    uint32_t previous_tick = 0;
    uint64_t previous_receipt_ns = 0;
    uint64_t stable_since_ns = 0;
    bool stable_down = false;
    for (int poll = 0; poll < config.down_max_polls; ++poll) {
        if (cancelled()) return fail("startup handoff cancelled while waiting for down pose");
        const Go2StartupSample sample = hooks.sample_lowstate();
        const uint64_t now = hooks.now_ns();
        const bool fresh = sample.receipt_ns != 0 && sample.receipt_ns <= now &&
                           now - sample.receipt_ns <= config.lowstate_max_age_ns;
        if (!have_tick || static_cast<int32_t>(sample.tick - previous_tick) > 0) {
            have_tick = true;
            previous_tick = sample.tick;
            if (previous_receipt_ns != 0 &&
                (sample.receipt_ns <= previous_receipt_ns ||
                 sample.receipt_ns - previous_receipt_ns > config.lowstate_max_age_ns))
                stable_since_ns = 0;
            previous_receipt_ns = sample.receipt_ns;
            if (fresh && go2_startup_sample_is_down(sample, config)) {
                if (stable_since_ns == 0) stable_since_ns = now;
                if (now - stable_since_ns >= config.down_stable_duration_ns) {
                    stable_down = true;
                    break;
                }
            } else {
                stable_since_ns = 0;
            }
        } else if (sample.tick != previous_tick) {
            return fail("LowState tick regressed while waiting for down pose");
        } else if (!fresh || (stable_since_ns != 0 && now - sample.receipt_ns > config.lowstate_max_age_ns)) {
            stable_since_ns = 0;
        }
        wait();
    }
    if (!stable_down) {
        return fail(sport_was_active
                        ? "StandDown did not reach a stable down pose before timeout"
                        : "sport mode is inactive, but the robot is not in a stable down pose");
    }

    form.clear();
    mode.clear();
    const int pre_release_mode_result = hooks.check_mode(form, mode);
    if (pre_release_mode_result != 0)
        return fail("CheckMode failed before ReleaseMode: " + std::to_string(pre_release_mode_result));
    if (cancelled()) return fail("startup handoff cancelled before ReleaseMode");

    if (!mode.empty()) {
        if (!sport_was_active)
            return fail("sport mode became active during startup; restart after placing the robot down");
        if (mode != initial_mode || form != initial_form)
            return fail("active motion mode changed during StandDown; refusing to release a different owner");
        if (!sample_is_fresh_and_down())
            return fail("down pose became stale or unstable before ReleaseMode");
        if (!hooks.release_mode) return fail("ReleaseMode hook is missing");
        const int release_result = hooks.release_mode();
        if (release_result != 0)
            return fail("ReleaseMode failed: " + std::to_string(release_result));
        if (cancelled()) return fail("startup handoff cancelled after ReleaseMode");
    }

    bool mode_inactive = false;
    for (int poll = 0; poll < config.release_max_polls; ++poll) {
        if (cancelled()) return fail("startup handoff cancelled while verifying released mode");
        form.clear();
        mode.clear();
        const int mode_result = hooks.check_mode(form, mode);
        if (mode_result != 0)
            return fail("CheckMode failed while verifying released mode: " + std::to_string(mode_result));
        if (mode.empty()) {
            mode_inactive = true;
            break;
        }
        wait();
    }
    if (!mode_inactive) return fail("sport mode remained active after startup handoff timeout");

    int quiet_samples = 0;
    for (int poll = 0; poll < config.quiet_max_polls; ++poll) {
        if (cancelled()) return fail("startup handoff cancelled while checking rt/lowcmd");
        if (hooks.competing_lowcmd_active()) quiet_samples = 0;
        else if (++quiet_samples >= config.quiet_stable_samples)
            break;
        wait();
    }
    if (quiet_samples < config.quiet_stable_samples)
        return fail("another rt/lowcmd publisher remained active after sport mode release");

    if (cancelled()) return fail("startup handoff cancelled before LowCmd creation");
    form.clear();
    mode.clear();
    const int final_mode_result = hooks.check_mode(form, mode);
    if (final_mode_result != 0)
        return fail("final CheckMode failed: " + std::to_string(final_mode_result));
    if (!mode.empty()) return fail("sport mode reactivated before LowCmd creation");
    if (!sample_is_fresh_and_down())
        return fail("down pose became stale or unstable before LowCmd creation");
    if (hooks.competing_lowcmd_active())
        return fail("another rt/lowcmd publisher appeared before LowCmd creation");
    if (cancelled()) return fail("startup handoff cancelled before LowCmd creation");
    return {true, {}};
}
