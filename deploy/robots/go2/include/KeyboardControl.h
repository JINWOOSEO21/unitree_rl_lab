#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <vector>

enum class Go2RuntimeState { Passive, Stand, Policy, StandDown };
enum class Go2StateRequest { Passive, Stand, Policy, StandDown };

struct Go2KeyboardAxes
{
    float speed = 0.0f;
    float turn = 0.0f;
    bool relative_heading = false;
    float heading_offset = 0.0f;
};

struct Go2PolicyReadiness
{
    bool stand_complete = false;
    bool scan_fresh_and_valid = false;
    bool upright = false;
    bool gyro_bias_valid = false;
};

inline std::optional<Go2RuntimeState> go2_route_request(
    Go2RuntimeState current, Go2StateRequest request, Go2PolicyReadiness ready = {})
{
    switch (current) {
    case Go2RuntimeState::Passive:
        if (request == Go2StateRequest::Stand) return Go2RuntimeState::Stand;
        break;
    case Go2RuntimeState::Stand:
        if (request == Go2StateRequest::StandDown && ready.stand_complete && ready.upright)
            return Go2RuntimeState::StandDown;
        if (request == Go2StateRequest::Passive) return Go2RuntimeState::Passive;
        if (request == Go2StateRequest::Policy && ready.stand_complete &&
            ready.scan_fresh_and_valid && ready.upright && ready.gyro_bias_valid)
            return Go2RuntimeState::Policy;
        break;
    case Go2RuntimeState::StandDown:
        if (request == Go2StateRequest::Passive) return Go2RuntimeState::Passive;
        if (request == Go2StateRequest::Stand) return Go2RuntimeState::Stand;
        break;
    case Go2RuntimeState::Policy:
        if (request == Go2StateRequest::Passive) return Go2RuntimeState::Passive;
        if (request == Go2StateRequest::Stand) return Go2RuntimeState::Stand;
        break;
    }
    return std::nullopt;
}

inline float go2_pose_lerp(float measured, float target, double elapsed_s, double duration_s = 2.0)
{
    const float alpha = static_cast<float>(std::clamp(elapsed_s / duration_s, 0.0, 1.0));
    return measured + alpha * (target - measured);
}

// Quintic blend: zero target velocity and acceleration at both endpoints.
inline float go2_pose_smooth(float measured, float target, double elapsed_s, double duration_s)
{
    const double t = std::clamp(elapsed_s / duration_s, 0.0, 1.0);
    const double alpha = t*t*t*(10.0 + t*(-15.0 + 6.0*t));
    return measured + static_cast<float>(alpha) * (target-measured);
}

inline bool go2_pose_within_tolerance(const std::vector<float>& measured,
                                      const std::vector<float>& target, float tolerance)
{
    if (measured.size() != target.size() || measured.empty() || !std::isfinite(tolerance) || tolerance < 0)
        return false;
    for (size_t i = 0; i < measured.size(); ++i)
        if (!std::isfinite(measured[i]) || !std::isfinite(target[i]) ||
            std::abs(measured[i] - target[i]) > tolerance)
            return false;
    return true;
}

inline bool go2_scan_values_valid(const std::vector<float>& values)
{
    return values.size() == 132 && std::all_of(values.begin(), values.end(), [](float value) {
        return std::isfinite(value) && value >= -1.0f && value <= 1.0f;
    });
}

inline bool go2_down_joint_settled(float measured, float target, float velocity, float tolerance)
{
    return std::isfinite(measured) && std::isfinite(target) && std::isfinite(velocity) &&
           std::isfinite(tolerance) && tolerance >= 0 &&
           std::abs(measured-target) <= tolerance && std::abs(velocity) <= 0.2f;
}

// Only advancing source samples accrue the stationary dwell; gaps restart it.
class Go2StandDownGate
{
public:
    void reset() { first_.reset(); last_.reset(); ready_ = false; }
    void update(uint32_t tick, bool settled)
    {
        if (!settled || (last_ && (tick < *last_ || tick-*last_ > 100))) {
            first_.reset(); ready_ = false;
        }
        if (settled && (!last_ || tick != *last_)) {
            if (!first_) first_ = tick;
            ready_ = tick >= *first_ && tick-*first_ >= 500;
        }
        last_ = tick;
    }
    bool ready() const { return ready_; }
private:
    std::optional<uint32_t> first_, last_;
    bool ready_ = false;
};

class Go2KeyboardControl
{
public:
    void set_state(Go2RuntimeState state)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        state_ = state;
        heading_keys_active_ = false;
        request_.reset();
        if (state != Go2RuntimeState::Policy) axes_ = {};
    }

    Go2RuntimeState state() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return state_;
    }

    Go2KeyboardAxes axes() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return axes_;
    }

    // X11 supplies physical key state, including releases (terminal bytes cannot).
    void set_heading_keys(bool left, bool right)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (state_ != Go2RuntimeState::Policy) return;
        if (left || right) {
            axes_.relative_heading = true;
            axes_.turn = 0;
        }
        if (left || right || heading_keys_active_)
            axes_.heading_offset = (static_cast<int>(left) - static_cast<int>(right)) * 0.2617993878f;
        heading_keys_active_ = left || right;
    }

    void handle_key(char key)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        switch (key) {
        case 'i': axes_.speed = clamp_axis(axes_.speed + 0.25f); break;
        case 'k': axes_.speed = clamp_axis(axes_.speed - 0.25f); break;
        case 'w':
            axes_.relative_heading = true;
            axes_.heading_offset = 0;
            axes_.turn = 0;
            heading_keys_active_ = false;
            break;
        case 'q':
        case 'e': {
            constexpr float pi = 3.14159265358979323846f;
            axes_.relative_heading = true;
            axes_.turn = 0;
            heading_keys_active_ = false;
            axes_.heading_offset = std::remainder(
                axes_.heading_offset + (key == 'q' ? 1.0f : -1.0f) * pi / 18.0f,
                2.0f * pi);
            break;
        }
        case ' ':
            heading_keys_active_ = false;
            axes_ = {};
            if (state_ == Go2RuntimeState::Policy) request_ = Go2StateRequest::Stand;
            break;
        case '0': request_ = Go2StateRequest::Passive; break;
        case '1': request_ = Go2StateRequest::Stand; break;
        case '2': request_ = Go2StateRequest::Policy; break;
        case '3':
            if (request_ != Go2StateRequest::Passive) request_ = Go2StateRequest::StandDown;
            break;
        default: break;
        }
    }

    void foreground_lost()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        axes_ = {};
        heading_keys_active_ = false;
        request_.reset();
        if (state_ == Go2RuntimeState::Policy) request_ = Go2StateRequest::Stand;
    }

    bool consume_transition(Go2RuntimeState target, Go2PolicyReadiness ready = {})
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!request_) return false;
        const auto routed = go2_route_request(state_, *request_, ready);
        if (!routed) {
            request_.reset();
            return false;
        }
        if (*routed != target) return false;
        request_.reset();
        return true;
    }

    std::optional<Go2StateRequest> pending_request() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return request_;
    }

    bool consume_request(Go2StateRequest request)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (request_ != request) return false;
        request_.reset();
        return true;
    }

private:
    static float clamp_axis(float value) { return std::clamp(value, -1.0f, 1.0f); }

    mutable std::mutex mutex_;
    Go2RuntimeState state_ = Go2RuntimeState::Passive;
    Go2KeyboardAxes axes_{};
    bool heading_keys_active_ = false;
    std::optional<Go2StateRequest> request_;
};

inline std::shared_ptr<Go2KeyboardControl> go2_keyboard_control;
