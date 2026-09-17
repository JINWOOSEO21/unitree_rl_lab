#pragma once

#include <atomic>
#include <cstdint>
#include "KeyboardControl.h"

enum class Go2ShutdownAction { Hold, Stand, Down };

// request()/complete() cross threads. step() is owned only by the FSM thread.
class Go2ShutdownControl
{
public:
    void request() { requested_.store(true); }
    bool requested() const { return requested_.load(); }
    bool complete() const { return complete_.load(); }

    Go2ShutdownAction step(Go2RuntimeState state, uint32_t tick, uint64_t now_ns,
                           bool fresh, bool joints_valid, bool upright,
                           bool stand_ready, bool motion_complete, bool measured_down)
    {
        if (!requested() || complete()) return Go2ShutdownAction::Hold;
        if (state != last_state_) {
            down_since_ns_ = 0;
            last_state_ = state;
        }
        const bool advanced = have_tick_ && static_cast<int32_t>(tick - last_tick_) > 0;
        const bool regressed = have_tick_ && static_cast<int32_t>(tick - last_tick_) < 0;
        const bool continuous = have_tick_ && now_ns >= last_advance_ns_ &&
                                now_ns - last_advance_ns_ <= 100'000'000;
        if (!have_tick_ || advanced || regressed) {
            last_tick_ = tick;
            last_advance_ns_ = now_ns;
            have_tick_ = true;
        }
        const bool healthy = fresh && joints_valid && continuous && !regressed;
        if (!healthy) {
            down_since_ns_ = 0;
            return Go2ShutdownAction::Hold;
        }
        if (state == Go2RuntimeState::Policy) {
            down_since_ns_ = 0;
            // Existing Policy fault checks run before this shutdown route.
            return Go2ShutdownAction::Stand;
        } else if (state == Go2RuntimeState::Stand) {
            down_since_ns_ = 0;
            if (stand_ready && upright) return Go2ShutdownAction::Down;
        } else if (measured_down && motion_complete && upright) {
            if (advanced) {
                if (!down_since_ns_) down_since_ns_ = now_ns;
                // Confirm 500 ms settling, then hold another full second.
                if (now_ns - down_since_ns_ >= 1'500'000'000) complete_.store(true);
            }
        } else {
            down_since_ns_ = 0;
        }
        return Go2ShutdownAction::Hold;
    }

private:
    std::atomic<bool> requested_{false}, complete_{false};
    Go2RuntimeState last_state_ = Go2RuntimeState::Passive;
    bool have_tick_ = false;
    uint32_t last_tick_ = 0;
    uint64_t last_advance_ns_ = 0, down_since_ns_ = 0;
};
