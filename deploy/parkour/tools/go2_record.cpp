// Subscriber-only bounded Go2 DDS recorder. It never creates publishers or RPC clients.
// Usage: go2_record <interface> <output_dir> [seconds=15] [domain=0]
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/idl/ros2/PointCloud2_.hpp>

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>

namespace {
using Clock = std::chrono::steady_clock;
using Low = unitree_go::msg::dds_::LowState_;
using Sport = unitree_go::msg::dds_::SportModeState_;
using Cloud = sensor_msgs::msg::dds_::PointCloud2_;
constexpr uint64_t kMaxCloudBytes = 256ULL * 1024ULL * 1024ULL;

template <typename Range>
void array_json(std::ostream& out, const Range& values)
{
    out << '[';
    bool first = true;
    for (const auto& value : values) {
        if (!first) out << ',';
        first = false;
        out << +value;
    }
    out << ']';
}

void string_json(std::ostream& out, const std::string& value)
{
    out << '"';
    for (const unsigned char c : value) {
        switch (c) {
        case '"': out << "\\\""; break;
        case '\\': out << "\\\\"; break;
        case '\b': out << "\\b"; break;
        case '\f': out << "\\f"; break;
        case '\n': out << "\\n"; break;
        case '\r': out << "\\r"; break;
        case '\t': out << "\\t"; break;
        default:
            if (c < 0x20) {
                out << "\\u00" << std::hex << std::setw(2) << std::setfill('0') << +c
                    << std::dec << std::setfill(' ');
            } else {
                out << c;
            }
        }
    }
    out << '"';
}

uint64_t steady_ns(const Clock::time_point start)
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now() - start).count();
}

void imu_json(std::ostream& out, const unitree_go::msg::dds_::IMUState_& imu)
{
    out << "{\"quaternion\":"; array_json(out, imu.quaternion());
    out << ",\"gyroscope\":"; array_json(out, imu.gyroscope());
    out << ",\"accelerometer\":"; array_json(out, imu.accelerometer());
    out << ",\"rpy\":"; array_json(out, imu.rpy());
    out << ",\"temperature\":" << +imu.temperature() << '}';
}

void low_json(std::ostream& out, uint64_t time_ns, const Low& state)
{
    out << std::setprecision(9) << "{\"steady_ns\":" << time_ns << ",\"head\":";
    array_json(out, state.head());
    out << ",\"level_flag\":" << +state.level_flag()
        << ",\"frame_reserve\":" << +state.frame_reserve() << ",\"sn\":";
    array_json(out, state.sn()); out << ",\"version\":"; array_json(out, state.version());
    out << ",\"bandwidth\":" << state.bandwidth() << ",\"imu_state\":";
    imu_json(out, state.imu_state());
    out << ",\"motor_state\":[";
    for (size_t i = 0; i < state.motor_state().size(); ++i) {
        const auto& motor = state.motor_state()[i];
        if (i) out << ',';
        out << "{\"mode\":" << +motor.mode() << ",\"q\":" << motor.q()
            << ",\"dq\":" << motor.dq() << ",\"ddq\":" << motor.ddq()
            << ",\"tau_est\":" << motor.tau_est() << ",\"q_raw\":" << motor.q_raw()
            << ",\"dq_raw\":" << motor.dq_raw() << ",\"ddq_raw\":" << motor.ddq_raw()
            << ",\"temperature\":" << +motor.temperature() << ",\"lost\":" << motor.lost()
            << ",\"reserve\":"; array_json(out, motor.reserve()); out << '}';
    }
    const auto& bms = state.bms_state();
    out << "],\"bms_state\":{\"version_high\":" << +bms.version_high()
        << ",\"version_low\":" << +bms.version_low() << ",\"status\":" << +bms.status()
        << ",\"soc\":" << +bms.soc() << ",\"current\":" << bms.current()
        << ",\"cycle\":" << bms.cycle() << ",\"bq_ntc\":";
    array_json(out, bms.bq_ntc()); out << ",\"mcu_ntc\":"; array_json(out, bms.mcu_ntc());
    out << ",\"cell_vol\":"; array_json(out, bms.cell_vol());
    out << "},\"foot_force\":"; array_json(out, state.foot_force());
    out << ",\"foot_force_est\":"; array_json(out, state.foot_force_est());
    out << ",\"tick\":" << state.tick() << ",\"wireless_remote\":";
    array_json(out, state.wireless_remote());
    out << ",\"bit_flag\":" << +state.bit_flag() << ",\"adc_reel\":" << state.adc_reel()
        << ",\"temperature_ntc1\":" << +state.temperature_ntc1()
        << ",\"temperature_ntc2\":" << +state.temperature_ntc2()
        << ",\"power_v\":" << state.power_v() << ",\"power_a\":" << state.power_a()
        << ",\"fan_frequency\":";
    array_json(out, state.fan_frequency());
    out << ",\"reserve\":" << state.reserve() << ",\"crc\":" << state.crc() << "}\n";
}

void sport_json(std::ostream& out, uint64_t time_ns, const Sport& state)
{
    out << std::setprecision(9) << "{\"steady_ns\":" << time_ns
        << ",\"stamp\":{\"sec\":" << state.stamp().sec()
        << ",\"nanosec\":" << state.stamp().nanosec() << "},\"error_code\":"
        << state.error_code() << ",\"imu_state\":";
    imu_json(out, state.imu_state());
    out << ",\"mode\":" << +state.mode() << ",\"progress\":" << state.progress()
        << ",\"gait_type\":" << +state.gait_type()
        << ",\"foot_raise_height\":" << state.foot_raise_height() << ",\"position\":";
    array_json(out, state.position()); out << ",\"body_height\":" << state.body_height();
    out << ",\"velocity\":"; array_json(out, state.velocity());
    out << ",\"yaw_speed\":" << state.yaw_speed() << ",\"range_obstacle\":";
    array_json(out, state.range_obstacle()); out << ",\"foot_force\":";
    array_json(out, state.foot_force()); out << ",\"foot_position_body\":";
    array_json(out, state.foot_position_body()); out << ",\"foot_speed_body\":";
    array_json(out, state.foot_speed_body()); out << ",\"path_point\":[";
    for (size_t i = 0; i < state.path_point().size(); ++i) {
        const auto& point = state.path_point()[i];
        if (i) out << ',';
        out << "{\"t_from_start\":" << point.t_from_start() << ",\"x\":" << point.x()
            << ",\"y\":" << point.y() << ",\"yaw\":" << point.yaw()
            << ",\"vx\":" << point.vx() << ",\"vy\":" << point.vy()
            << ",\"vyaw\":" << point.vyaw() << '}';
    }
    out << "]}\n";
}

void cloud_json(std::ostream& out, uint64_t time_ns, const Cloud& cloud,
                uint64_t offset, uint64_t size)
{
    out << "{\"steady_ns\":" << time_ns << ",\"stamp\":{\"sec\":"
        << cloud.header().stamp().sec() << ",\"nanosec\":"
        << cloud.header().stamp().nanosec() << "},\"frame_id\":";
    string_json(out, cloud.header().frame_id());
    out << ",\"height\":" << cloud.height() << ",\"width\":" << cloud.width()
        << ",\"fields\":[";
    for (size_t i = 0; i < cloud.fields().size(); ++i) {
        const auto& field = cloud.fields()[i];
        if (i) out << ',';
        out << "{\"name\":"; string_json(out, field.name());
        out << ",\"offset\":" << field.offset() << ",\"datatype\":" << +field.datatype()
            << ",\"count\":" << field.count() << '}';
    }
    out << "],\"is_bigendian\":" << (cloud.is_bigendian() ? "true" : "false")
        << ",\"point_step\":" << cloud.point_step() << ",\"row_step\":" << cloud.row_step()
        << ",\"is_dense\":" << (cloud.is_dense() ? "true" : "false")
        << ",\"binary_offset\":" << offset << ",\"binary_size\":" << size << "}\n";
}
}  // namespace

int main(int argc, char** argv)
{
    if (argc < 3 || argc > 5 || std::string(argv[1]) == "--help") {
        std::cout << "Usage: go2_record <interface> <output_dir> [seconds=15] [domain=0]\n"
                  << "SUBSCRIBER ONLY: records lowstate, sportmodestate and cloud; sends no commands.\n";
        return argc == 2 ? 0 : 1;
    }
    try {
        const auto integer = [](const char* value) {
            size_t used = 0;
            const int result = std::stoi(value, &used);
            if (used != std::string(value).size()) throw std::invalid_argument("Expected integer");
            return result;
        };
        const int seconds = argc > 3 ? integer(argv[3]) : 15;
        const int domain = argc > 4 ? integer(argv[4]) : 0;
        if (seconds < 1 || seconds > 60 || domain < 0 || domain > 232)
            throw std::invalid_argument("seconds: 1..60; domain: 0..232");
        const std::filesystem::path output(argv[2]);
        if (std::filesystem::exists(output)) throw std::runtime_error("output_dir already exists");
        std::filesystem::create_directories(output);
        std::ofstream low_file(output / "lowstate.jsonl");
        std::ofstream sport_file(output / "sportmodestate.jsonl");
        std::ofstream cloud_meta(output / "cloud.jsonl");
        std::ofstream cloud_bin(output / "cloud.bin", std::ios::binary);
        if (!low_file || !sport_file || !cloud_meta || !cloud_bin)
            throw std::runtime_error("failed to open output files");

        const auto start = Clock::now();
        std::mutex low_mutex, sport_mutex, cloud_mutex;
        std::atomic<uint64_t> low_count{0}, sport_count{0}, cloud_count{0}, cloud_bytes{0};
        std::atomic<bool> disk_limit{false};
        unitree::robot::ChannelFactory::Instance()->Init(domain, argv[1]);
        unitree::robot::ChannelSubscriber<Low> low_sub("rt/lowstate");
        unitree::robot::ChannelSubscriber<Sport> sport_sub("rt/sportmodestate");
        unitree::robot::ChannelSubscriber<Cloud> cloud_sub("rt/utlidar/cloud");
        low_sub.InitChannel([&](const void* message) {
            const uint64_t time_ns = steady_ns(start);
            std::lock_guard<std::mutex> lock(low_mutex);
            low_json(low_file, time_ns, *static_cast<const Low*>(message));
            ++low_count;
        }, 1);
        sport_sub.InitChannel([&](const void* message) {
            const uint64_t time_ns = steady_ns(start);
            std::lock_guard<std::mutex> lock(sport_mutex);
            sport_json(sport_file, time_ns, *static_cast<const Sport*>(message));
            ++sport_count;
        }, 1);
        cloud_sub.InitChannel([&](const void* message) {
            const uint64_t time_ns = steady_ns(start);
            std::lock_guard<std::mutex> lock(cloud_mutex);
            const auto& cloud = *static_cast<const Cloud*>(message);
            const uint64_t size = cloud.data().size();
            const uint64_t offset = cloud_bytes.load();
            if (size > kMaxCloudBytes - offset) {
                disk_limit = true;
                return;
            }
            cloud_bin.write(reinterpret_cast<const char*>(cloud.data().data()), size);
            cloud_json(cloud_meta, time_ns, cloud, offset, size);
            cloud_bytes += size;
            ++cloud_count;
        }, 1);
        std::cout << "SUBSCRIBE ONLY interface=" << argv[1] << " domain=" << domain
                  << " duration=" << seconds << "s output=" << output << '\n';
        std::this_thread::sleep_for(std::chrono::seconds(seconds));
        low_sub.CloseChannel(); sport_sub.CloseChannel(); cloud_sub.CloseChannel();
        {
            std::lock_guard<std::mutex> lock(low_mutex);
            low_file.flush();
        }
        {
            std::lock_guard<std::mutex> lock(sport_mutex);
            sport_file.flush();
        }
        {
            std::lock_guard<std::mutex> lock(cloud_mutex);
            cloud_meta.flush(); cloud_bin.flush();
        }
        std::ofstream manifest(output / "manifest.json");
        manifest << "{\"format_version\":1,\"subscriber_only\":true,\"interface\":";
        string_json(manifest, argv[1]);
        manifest << ",\"domain\":" << domain << ",\"requested_duration_s\":" << seconds
                 << ",\"timestamp_semantics\":\"steady nanoseconds since recorder start, sampled at callback entry before locking or I/O\""
                 << ",\"lowstate_count\":" << low_count.load() << ",\"sportmodestate_count\":"
                 << sport_count.load() << ",\"cloud_count\":" << cloud_count.load()
                 << ",\"cloud_bytes\":" << cloud_bytes.load() << ",\"cloud_byte_limit\":"
                 << kMaxCloudBytes << ",\"cloud_limit_hit\":" << (disk_limit ? "true" : "false")
                 << "}\n";
        manifest.close();
        std::cout << "counts lowstate=" << low_count.load()
                  << " sportmodestate=" << sport_count.load()
                  << " cloud=" << cloud_count.load() << " cloud_bytes=" << cloud_bytes.load() << '\n';
        if (!low_file || !sport_file || !cloud_meta || !cloud_bin || !manifest)
            throw std::runtime_error("write failure");
        if (!low_count || !sport_count || !cloud_count) {
            std::cerr << "NO DATA on one or more required topics\n";
            return 2;
        }
        return disk_limit ? 3 : 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
