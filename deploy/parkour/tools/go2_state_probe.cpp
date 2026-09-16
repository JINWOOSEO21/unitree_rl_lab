// Subscriber-only Go2 diagnostics. No motor publisher or service client.
// Usage: go2_state_probe <interface> [seconds=10] [domain=0]
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <algorithm>
#include <chrono>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

int main(int argc, char** argv)
{
    if (argc < 2 || argc > 4 || std::string(argv[1]) == "--help") {
        std::cout << "Usage: go2_state_probe <interface> [seconds=10] [domain=0]\n"
                  << "Subscribes only to rt/lowstate; no control commands.\n";
        return argc == 2 ? 0 : 1;
    }
    try {
        const auto integer = [](const char* value) {
            size_t used = 0;
            int result = std::stoi(value, &used);
            if (used != std::string(value).size())
                throw std::invalid_argument("Expected an integer");
            return result;
        };
        const int seconds = argc > 2 ? integer(argv[2]) : 10;
        const int domain = argc > 3 ? integer(argv[3]) : 0;
        if (seconds < 1 || seconds > 3600 || domain < 0 || domain > 232)
            throw std::invalid_argument("seconds: 1..3600; domain: 0..232");

        using State = unitree_go::msg::dds_::LowState_;
        using Clock = std::chrono::steady_clock;
        std::mutex mutex;
        State latest{};
        uint64_t count = 0;
        Clock::time_point first{}, last{};
        double max_gap = 0;
        unitree::robot::ChannelFactory::Instance()->Init(domain, argv[1]);
        unitree::robot::ChannelSubscriber<State> sub("rt/lowstate");
        sub.InitChannel([&](const void* message) {
            const auto now = Clock::now();
            std::lock_guard<std::mutex> guard(mutex);
            if (count) max_gap = std::max(max_gap, std::chrono::duration<double>(now - last).count());
            else first = now;
            latest = *static_cast<const State*>(message);
            last = now;
            ++count;
        }, 1);
        std::cout << "SUBSCRIBE ONLY: rt/lowstate interface=" << argv[1]
                  << " domain=" << domain << " duration=" << seconds << "s\n"
                  << "DDS reception alone does not identify a physical robot; exclude simulator publishers.\n";
        for (int i = 0; i < seconds; ++i) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            std::lock_guard<std::mutex> guard(mutex);
            if (!count) {
                std::cout << "Waiting: no lowstate received (" << i + 1 << "s)\n" << std::flush;
                continue;
            }
            const double span = std::chrono::duration<double>(last - first).count();
            std::cout << "count=" << count << " callback_Hz=" << (span > 0 ? (count - 1) / span : 0)
                      << " age_ms=" << std::chrono::duration<double, std::milli>(Clock::now() - last).count()
                      << " max_callback_gap_ms=" << max_gap * 1000 << " tick=" << latest.tick()
                      << " sn_raw=" << latest.sn()[0] << ',' << latest.sn()[1]
                      << " version_raw=" << latest.version()[0] << ',' << latest.version()[1]
                      << " battery_soc_raw=" << unsigned(latest.bms_state().soc())
                      << " voltage=" << latest.power_v() << '\n';
            std::cout << "q_sdk[0:12]=";
            for (int j = 0; j < 12; ++j) std::cout << latest.motor_state()[j].q() << ' ';
            std::cout << "\ndq_sdk[0:12]=";
            for (int j = 0; j < 12; ++j) std::cout << latest.motor_state()[j].dq() << ' ';
            std::cout << "\nimu_quaternion_raw=";
            for (auto v : latest.imu_state().quaternion()) std::cout << v << ' ';
            std::cout << " gyro=";
            for (auto v : latest.imu_state().gyroscope()) std::cout << v << ' ';
            std::cout << " foot_force_raw=";
            for (auto v : latest.foot_force()) std::cout << v << ' ';
            std::cout << '\n' << std::flush;
        }
        sub.CloseChannel();
        std::lock_guard<std::mutex> guard(mutex);
        if (!count) {
            std::cerr << "NO DATA: check cable, interface, subnet, domain and robot state.\n";
            return 2;
        }
        if (Clock::now() - last > std::chrono::seconds(1)) {
            std::cerr << "STALE: no lowstate received in the last second.\n";
            return 3;
        }
        return 0;
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
