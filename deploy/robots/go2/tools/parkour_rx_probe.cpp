// Subscriber-only bridge output diagnostics: no publisher, no LowCmd, no service client.
// Usage: go2_parkour_rx_probe <interface> [seconds=30] [domain=0]
//
// Step C-3: confirms the notebook receives what the Jetson bridge publishes.
// tools/scandots_probe.cpp cannot do this — it is pinned to domain 179 on "lo" and the
// rt/parkour/test_scandots loopback topic. This one reads the live rt/parkour/scandots
// and rt/parkour/gyro_bias with the same subscribers go2_ctrl uses.

#include <unitree/robot/channel/channel_factory.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <csignal>
#include <string>
#include <thread>
#include <vector>

#include "GyroBiasSubscriber.h"
#include "parkour/scandots.h"

namespace
{
volatile std::sig_atomic_t keep_running = 1;
void stop_on_signal(int) { keep_running = 0; }

int as_int(const char* text, const char* what, int low, int high)
{
    size_t used = 0;
    const int value = std::stoi(text, &used);
    if (used != std::string(text).size() || value < low || value > high)
        throw std::invalid_argument(std::string(what) + ": expected " + std::to_string(low) +
                                    ".." + std::to_string(high));
    return value;
}
}  // namespace

int main(int argc, char** argv)
{
    if (argc < 2 || argc > 4 || std::string(argv[1]) == "--help") {
        std::puts("Usage: go2_parkour_rx_probe <interface> [seconds=30] [domain=0]\n"
                  "Subscribes only to rt/parkour/scandots and rt/parkour/gyro_bias.\n"
                  "No publisher is created and nothing is sent to the robot.");
        return argc == 2 ? 0 : 1;
    }
    try {
        const int seconds = argc > 2 ? as_int(argv[2], "seconds", 1, 3600) : 30;
        const int domain = argc > 3 ? as_int(argv[3], "domain", 0, 232) : 0;

        unitree::robot::ChannelFactory::Instance()->Init(domain, argv[1]);
        parkour::ScandotsSubscriber scandots;
        Go2GyroBiasSubscriber gyro_bias;

        std::printf("SUBSCRIBE ONLY: rt/parkour/scandots + rt/parkour/gyro_bias "
                    "interface=%s domain=%d duration=%ds\n",
                    argv[1], domain, seconds);
        std::printf("Bridge output only; this says nothing about the robot's own topics.\n");
        std::fflush(stdout);

        std::signal(SIGINT, stop_on_signal);
        std::signal(SIGTERM, stop_on_signal);

        using Clock = std::chrono::steady_clock;
        const auto start = Clock::now();
        uint64_t previous = scandots.count();
        auto previous_at = start;
        double max_gap_ms = 0.0;
        // A gap is only meaningful between two observed arrivals, so the first one seeds it.
        bool have_arrival = false;

        for (int second = 0; second < seconds && keep_running; ++second) {
            // 50 ms polling resolves a ~10 Hz stream well enough to expose stalls.
            for (int tick = 0; tick < 20 && keep_running; ++tick) {
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
                const uint64_t now_count = scandots.count();
                if (now_count == previous) continue;
                const auto now = Clock::now();
                if (have_arrival)
                    max_gap_ms = std::max(
                        max_gap_ms, std::chrono::duration<double, std::milli>(now - previous_at).count());
                have_arrival = true;
                previous = now_count;
                previous_at = now;
            }

            const double elapsed = std::chrono::duration<double>(Clock::now() - start).count();
            const uint64_t count = scandots.count();
            const double age_ms = scandots.last_age() * 1000.0;

            parkour::GyroBiasSample sample{};
            std::string reason;
            const bool bias_ok = gyro_bias.get(sample, &reason);

            std::printf("t=%4.0fs scandots count=%llu rate_hz=%.2f age_ms=%.1f max_gap_ms=%.1f bad=%llu",
                        elapsed, static_cast<unsigned long long>(count),
                        elapsed > 0 ? count / elapsed : 0.0,
                        age_ms > 1e8 ? -1.0 : age_ms, max_gap_ms,
                        static_cast<unsigned long long>(scandots.bad_size_count()));
            if (bias_ok)
                std::printf("  gyro_bias=[%.10f,%.10f,%.10f] session=%s seq=%llu tick=%llu\n",
                            sample.bias[0], sample.bias[1], sample.bias[2], sample.session.c_str(),
                            static_cast<unsigned long long>(sample.sequence),
                            static_cast<unsigned long long>(sample.tick));
            else
                std::printf("  gyro_bias=none (%s)\n", reason.c_str());
            std::fflush(stdout);
        }

        const uint64_t total = scandots.count();
        if (total == 0) {
            std::fprintf(stderr,
                         "NO SCANDOTS: bridge not publishing, wrong interface/domain, or "
                         "CYCLONEDDS_URI pinning DDS to another interface.\n");
            return 2;
        }
        if (scandots.last_age() > 1.0) {
            std::fprintf(stderr, "STALE: no scandots in the last second.\n");
            return 3;
        }
        return 0;
    } catch (const std::exception& error) {
        std::fprintf(stderr, "%s\n", error.what());
        return 1;
    }
}
