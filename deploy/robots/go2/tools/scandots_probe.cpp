// Read-only DDS probe for the Parkour scandots contract.
// Fixed isolation: loopback interface, domain 179, test topic. No robot/FSM/LowCmd APIs.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>

#include <unitree/robot/channel/channel_factory.hpp>

#include "parkour/scandots.h"

namespace
{
double argument_double(int argc, char** argv, const std::string& name, double fallback)
{
    for (int i = 1; i + 1 < argc; ++i)
        if (argv[i] == name) return std::stod(argv[i + 1]);
    return fallback;
}

void emit(const parkour::ScandotsSubscriber& subscriber, const char* event, double elapsed)
{
    auto values = subscriber.get();
    std::vector<float> fresh_values;
    const uint64_t count = subscriber.count();
    const double age = subscriber.last_age();
    const bool ready = subscriber.get_if_fresh(1.0, fresh_values);
    if (ready) values = std::move(fresh_values);
    const auto [minimum, maximum] = std::minmax_element(values.begin(), values.end());
    std::cout << std::fixed << std::setprecision(6)
              << "{\"event\":\"" << event << "\",\"elapsed_s\":" << elapsed
              << ",\"ready\":" << (ready ? "true" : "false")
              << ",\"count_since_invalidation\":" << count
              << ",\"bad_count\":" << subscriber.bad_size_count()
              << ",\"age_s\":" << (count ? age : -1.0)
              << ",\"value_count\":" << values.size()
              << ",\"first\":" << values.front()
              << ",\"min\":" << *minimum << ",\"max\":" << *maximum << ",\"values\":[";
    for (size_t i = 0; i < values.size(); ++i) {
        if (i) std::cout << ',';
        std::cout << values[i];
    }
    std::cout << "]}\n" << std::flush;
}
}  // namespace

int main(int argc, char** argv)
{
    const double duration = argument_double(argc, argv, "--duration", 10.0);
    const double interval = argument_double(argc, argv, "--interval", 0.05);
    if (!std::isfinite(duration) || duration <= 0 || duration > 60 ||
        !std::isfinite(interval) || interval < 0.01 || interval > 1.0) {
        std::cerr << "duration must be in (0,60], interval in [0.01,1.0]\n";
        return 2;
    }

    constexpr int kDomain = 179;
    constexpr const char* kInterface = "lo";
    constexpr const char* kTopic = "rt/parkour/test_scandots";
    unitree::robot::ChannelFactory::Instance()->Init(kDomain, kInterface);
    parkour::ScandotsSubscriber subscriber(kTopic);

    using Clock = std::chrono::steady_clock;
    const auto start = Clock::now();
    uint64_t last_count = subscriber.count();
    uint64_t last_bad = subscriber.bad_size_count();
    emit(subscriber, "start", 0.0);
    while (std::chrono::duration<double>(Clock::now() - start).count() < duration) {
        std::this_thread::sleep_for(std::chrono::duration<double>(interval));
        const uint64_t count = subscriber.count();
        const uint64_t bad = subscriber.bad_size_count();
        if (count != last_count || bad != last_bad) {
            emit(subscriber, count == 0 && bad > last_bad ? "invalidated" : "valid",
                 std::chrono::duration<double>(Clock::now() - start).count());
            last_count = count;
            last_bad = bad;
        }
    }
    emit(subscriber, "final", std::chrono::duration<double>(Clock::now() - start).count());
}
