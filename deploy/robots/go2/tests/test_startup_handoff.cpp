#include "StartupHandoff.h"

#include <algorithm>
#include <cassert>
#include <limits>
#include <string>
#include <vector>

namespace
{
Go2StartupHandoffConfig config()
{
    Go2StartupHandoffConfig value;
    value.down_q.fill(1.0f);
    value.joint_tolerance.fill(0.25f);
    value.down_stable_duration_ns = 40'000'000;
    value.down_max_polls = 8;
    value.release_max_polls = 4;
    value.quiet_stable_samples = 3;
    value.quiet_max_polls = 6;
    return value;
}

Go2StartupSample down_sample(uint32_t tick)
{
    Go2StartupSample sample;
    sample.tick = tick;
    sample.q.fill(1.0f);
    sample.dq.fill(0.0f);
    sample.quaternion = {1.0f, 0.0f, 0.0f, 0.0f};
    return sample;
}

struct Fake
{
    std::vector<std::string> events;
    std::vector<std::string> modes{"mcf", "mcf", ""};
    size_t mode_index = 0;
    uint32_t tick = 0;
    bool noisy = false;
    bool standing = false;
    bool stale = false;
    bool cancelled = false;
    bool cancel_after_check = false;
    bool standing_after_release = false;
    bool regress_tick = false;
    uint64_t now_ns = 1;
    int stand_result = 0;
    int release_result = 0;
    int check_result = 0;
    int lowcmd_active_polls = 0;

    Go2StartupHandoffHooks hooks()
    {
        return {
            [this](std::string&, std::string& name) {
                events.push_back("check");
                if (cancel_after_check) cancelled = true;
                if (check_result) return check_result;
                name = modes[std::min(mode_index++, modes.size() - 1)];
                return 0;
            },
            [this] {
                events.push_back("stand_down");
                return stand_result;
            },
            [this] {
                events.push_back("release");
                return release_result;
            },
            [this] {
                ++tick;
                auto sample = down_sample(stale ? 1u : (regress_tick && tick == 2 ? 0u : tick));
                sample.receipt_ns = stale ? 1u : now_ns;
                if (standing || (standing_after_release &&
                                 std::find(events.begin(), events.end(), "release") != events.end()))
                    sample.q.fill(0.0f);
                if (noisy && tick == 2) sample.dq[0] = 1.0f;
                return sample;
            },
            [this] { return lowcmd_active_polls-- > 0; },
            [this] { return cancelled; },
            [this] { return now_ns; },
            [this] { now_ns += 20'000'000; },
        };
    }
};
}

int main()
{
    {
        auto cfg = config();
        cfg.down_q = {0.0f, 1.36f, -2.65f, 0.0f, 1.36f, -2.65f,
                      0.0f, 1.36f, -2.65f, 0.0f, 1.36f, -2.65f};
        cfg.joint_tolerance = {0.50f, 0.25f, 0.25f, 0.50f, 0.25f, 0.25f,
                               0.50f, 0.25f, 0.25f, 0.50f, 0.25f, 0.25f};
        auto lying = down_sample(1);
        lying.q = {-0.068f, 1.243f, -2.789f, 0.0866f, 1.239f, -2.751f,
                   -0.3776f, 1.2705f, -2.770f, 0.3821f, 1.270f, -2.756f};
        assert(go2_startup_sample_is_down(lying, cfg));
        auto standing = lying;
        standing.q = {0.0f, 0.70f, -1.45f, 0.0f, 0.70f, -1.45f,
                      0.0f, 0.70f, -1.45f, 0.0f, 0.70f, -1.45f};
        assert(!go2_startup_sample_is_down(standing, cfg));
        lying.quaternion = {0.7071067f, 0.7071067f, 0.0f, 0.0f};
        assert(!go2_startup_sample_is_down(lying, cfg));
    }
    {
        Fake fake;
        fake.lowcmd_active_polls = 1;
        const auto result = go2_perform_startup_handoff(config(), fake.hooks());
        assert(result.ready);
        const auto stand = std::find(fake.events.begin(), fake.events.end(), "stand_down");
        const auto release = std::find(fake.events.begin(), fake.events.end(), "release");
        assert(stand != fake.events.end() && release != fake.events.end() && stand < release);
    }
    {
        Fake fake;
        fake.stand_result = 7;
        const auto result = go2_perform_startup_handoff(config(), fake.hooks());
        assert(!result.ready);
        assert(std::find(fake.events.begin(), fake.events.end(), "release") == fake.events.end());
    }
    {
        Fake fake;
        fake.release_result = 8;
        const auto result = go2_perform_startup_handoff(config(), fake.hooks());
        assert(!result.ready);
    }
    for (const auto condition : {std::string("stale"), std::string("standing")}) {
        Fake fake;
        if (condition == "stale") fake.stale = true;
        else fake.standing = true;
        const auto result = go2_perform_startup_handoff(config(), fake.hooks());
        assert(!result.ready);
        assert(std::find(fake.events.begin(), fake.events.end(), "release") == fake.events.end());
    }
    {
        Fake fake;
        fake.regress_tick = true;
        const auto result = go2_perform_startup_handoff(config(), fake.hooks());
        assert(!result.ready);
        assert(result.error.find("regressed") != std::string::npos);
        assert(std::find(fake.events.begin(), fake.events.end(), "release") == fake.events.end());
    }
    {
        Fake fake;
        fake.noisy = true;
        const auto result = go2_perform_startup_handoff(config(), fake.hooks());
        assert(result.ready);
    }
    {
        Fake fake;
        fake.cancelled = true;
        const auto result = go2_perform_startup_handoff(config(), fake.hooks());
        assert(!result.ready);
        assert(std::find(fake.events.begin(), fake.events.end(), "release") == fake.events.end());
    }
    {
        Fake fake;
        fake.cancel_after_check = true;
        const auto result = go2_perform_startup_handoff(config(), fake.hooks());
        assert(!result.ready);
        assert(std::find(fake.events.begin(), fake.events.end(), "stand_down") == fake.events.end());
    }
    {
        Fake fake;
        fake.standing_after_release = true;
        const auto result = go2_perform_startup_handoff(config(), fake.hooks());
        assert(!result.ready);
    }
    {
        Fake fake;
        fake.modes = {"", "", ""};
        const auto result = go2_perform_startup_handoff(config(), fake.hooks());
        assert(result.ready);
        assert(std::find(fake.events.begin(), fake.events.end(), "stand_down") == fake.events.end());
        assert(std::find(fake.events.begin(), fake.events.end(), "release") == fake.events.end());
    }
    {
        Fake fake;
        fake.modes = {"mcf", "different-owner"};
        const auto result = go2_perform_startup_handoff(config(), fake.hooks());
        assert(!result.ready);
        assert(std::find(fake.events.begin(), fake.events.end(), "release") == fake.events.end());
    }
    {
        Fake fake;
        fake.modes = {"mcf", "mcf", "", "mcf"};
        const auto result = go2_perform_startup_handoff(config(), fake.hooks());
        assert(!result.ready);
        assert(result.error.find("reactivated") != std::string::npos);
    }
    {
        Fake fake;
        fake.modes = {"", "", ""};
        fake.standing = true;
        const auto result = go2_perform_startup_handoff(config(), fake.hooks());
        assert(!result.ready);
    }
    {
        Fake fake;
        fake.check_result = 9;
        const auto result = go2_perform_startup_handoff(config(), fake.hooks());
        assert(!result.ready);
        assert(std::find(fake.events.begin(), fake.events.end(), "stand_down") == fake.events.end());
    }
    {
        Fake fake;
        auto cfg = config();
        cfg.down_q[0] = std::numeric_limits<float>::quiet_NaN();
        const auto result = go2_perform_startup_handoff(cfg, fake.hooks());
        assert(!result.ready);
        assert(fake.events.empty());
    }
    {
        Fake fake;
        fake.lowcmd_active_polls = 100;
        const auto result = go2_perform_startup_handoff(config(), fake.hooks());
        assert(!result.ready);
    }
}
