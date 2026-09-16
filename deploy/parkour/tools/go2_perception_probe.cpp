// Subscriber-only Go2 perception diagnostics. No publishers or service clients.
// Usage: go2_perception_probe <interface> [seconds=10] [domain=0]
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/idl/ros2/PointCloud2_.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

namespace {
using Clock = std::chrono::steady_clock;
using Sport = unitree_go::msg::dds_::SportModeState_;
using Cloud = sensor_msgs::msg::dds_::PointCloud2_;

struct TopicTiming {
    uint64_t count = 0;
    Clock::time_point first{};
    Clock::time_point last{};
    double max_gap_ms = 0.0;

    void observe(Clock::time_point now)
    {
        if (count == 0) first = now;
        else max_gap_ms = std::max(max_gap_ms,
            std::chrono::duration<double, std::milli>(now - last).count());
        last = now;
        ++count;
    }

    double hz() const
    {
        const double span = std::chrono::duration<double>(last - first).count();
        return count > 1 && span > 0.0 ? (count - 1) / span : 0.0;
    }
};

struct CloudSummary {
    TopicTiming timing;
    int32_t stamp_sec = 0;
    uint32_t stamp_nsec = 0;
    std::string frame_id;
    uint32_t width = 0;
    uint32_t height = 0;
    uint32_t point_step = 0;
    uint32_t row_step = 0;
    size_t data_size = 0;
    bool big_endian = false;
    bool dense = false;
    std::string fields;
    bool layout_ok = false;
    bool xyz_layout_ok = false;
    bool packed_xyz12 = false;
    uint64_t finite_points = 0;
    uint64_t nonfinite_points = 0;
    std::array<double, 3> xyz_min{};
    std::array<double, 3> xyz_max{};
};

bool host_is_big_endian()
{
    const uint16_t value = 0x0102;
    return *reinterpret_cast<const uint8_t*>(&value) == 0x01;
}

template <typename T>
T read_scalar(const uint8_t* bytes, bool source_big_endian)
{
    std::array<uint8_t, sizeof(T)> raw{};
    std::copy(bytes, bytes + sizeof(T), raw.begin());
    if (source_big_endian != host_is_big_endian()) std::reverse(raw.begin(), raw.end());
    T value{};
    std::memcpy(&value, raw.data(), sizeof(T));
    return value;
}

bool read_coordinate(const uint8_t* bytes, uint8_t datatype, bool big_endian, double& value)
{
    using namespace sensor_msgs::msg::dds_::PointField_Constants;
    if (datatype == FLOAT32_) value = read_scalar<float>(bytes, big_endian);
    else if (datatype == FLOAT64_) value = read_scalar<double>(bytes, big_endian);
    else return false;
    return true;
}

size_t datatype_size(uint8_t datatype)
{
    using namespace sensor_msgs::msg::dds_::PointField_Constants;
    switch (datatype) {
    case INT8_: case UINT8_: return 1;
    case INT16_: case UINT16_: return 2;
    case INT32_: case UINT32_: case FLOAT32_: return 4;
    case FLOAT64_: return 8;
    default: return 0;
    }
}

std::string describe_fields(const Cloud& cloud)
{
    std::string result;
    for (const auto& field : cloud.fields()) {
        if (!result.empty()) result += ',';
        result += field.name() + '@' + std::to_string(field.offset()) + ':'
            + std::to_string(unsigned(field.datatype())) + 'x' + std::to_string(field.count());
    }
    return result;
}

CloudSummary summarize_cloud(const Cloud& cloud, const TopicTiming& timing)
{
    CloudSummary out;
    out.timing = timing;
    out.stamp_sec = cloud.header().stamp().sec();
    out.stamp_nsec = cloud.header().stamp().nanosec();
    out.frame_id = cloud.header().frame_id();
    out.width = cloud.width();
    out.height = cloud.height();
    out.point_step = cloud.point_step();
    out.row_step = cloud.row_step();
    out.data_size = cloud.data().size();
    out.big_endian = cloud.is_bigendian();
    out.dense = cloud.is_dense();
    out.fields = describe_fields(cloud);

    const uint64_t required_bytes = static_cast<uint64_t>(out.row_step) * out.height;
    const uint64_t minimum_row_bytes = static_cast<uint64_t>(out.point_step) * out.width;
    out.layout_ok = out.height > 0 && out.point_step > 0
        && out.row_step >= minimum_row_bytes && out.data_size >= required_bytes;

    std::array<uint32_t, 3> offsets{};
    std::array<uint8_t, 3> datatypes{};
    std::array<bool, 3> found{};
    const std::array<std::string, 3> names{"x", "y", "z"};
    for (const auto& field : cloud.fields()) {
        for (size_t axis = 0; axis < names.size(); ++axis) {
            if (field.name() == names[axis] && field.count() >= 1) {
                found[axis] = true;
                offsets[axis] = field.offset();
                datatypes[axis] = field.datatype();
            }
        }
    }
    out.xyz_layout_ok = out.layout_ok;
    for (size_t axis = 0; axis < 3; ++axis) {
        const size_t bytes = datatype_size(datatypes[axis]);
        out.xyz_layout_ok = out.xyz_layout_ok && found[axis]
            && (datatypes[axis] == sensor_msgs::msg::dds_::PointField_Constants::FLOAT32_
                || datatypes[axis] == sensor_msgs::msg::dds_::PointField_Constants::FLOAT64_)
            && offsets[axis] + bytes <= out.point_step;
        out.xyz_min[axis] = std::numeric_limits<double>::infinity();
        out.xyz_max[axis] = -std::numeric_limits<double>::infinity();
    }
    out.packed_xyz12 = out.xyz_layout_ok && out.height == 1 && out.point_step == 12
        && offsets[0] == 0 && offsets[1] == 4 && offsets[2] == 8
        && datatypes[0] == sensor_msgs::msg::dds_::PointField_Constants::FLOAT32_
        && datatypes[1] == sensor_msgs::msg::dds_::PointField_Constants::FLOAT32_
        && datatypes[2] == sensor_msgs::msg::dds_::PointField_Constants::FLOAT32_;
    if (!out.xyz_layout_ok) return out;

    for (uint32_t row = 0; row < out.height; ++row) {
        for (uint32_t col = 0; col < out.width; ++col) {
            const size_t base = static_cast<size_t>(row) * out.row_step
                + static_cast<size_t>(col) * out.point_step;
            std::array<double, 3> xyz{};
            bool finite = true;
            for (size_t axis = 0; axis < 3; ++axis) {
                if (!read_coordinate(cloud.data().data() + base + offsets[axis],
                                     datatypes[axis], out.big_endian, xyz[axis])
                    || !std::isfinite(xyz[axis])) finite = false;
            }
            if (!finite) {
                ++out.nonfinite_points;
                continue;
            }
            ++out.finite_points;
            for (size_t axis = 0; axis < 3; ++axis) {
                out.xyz_min[axis] = std::min(out.xyz_min[axis], xyz[axis]);
                out.xyz_max[axis] = std::max(out.xyz_max[axis], xyz[axis]);
            }
        }
    }
    return out;
}

template <typename T>
void print_array(const T& values)
{
    for (size_t i = 0; i < values.size(); ++i) {
        if (i) std::cout << ',';
        std::cout << values[i];
    }
}
}  // namespace

int main(int argc, char** argv)
{
    if (argc < 2 || argc > 4 || std::string(argv[1]) == "--help") {
        std::cout << "Usage: go2_perception_probe <interface> [seconds=10] [domain=0]\n"
                  << "Subscribes only to rt/sportmodestate and rt/utlidar/cloud; sends no commands.\n";
        return argc == 2 ? 0 : 1;
    }
    try {
        const auto integer = [](const char* value) {
            size_t used = 0;
            const int result = std::stoi(value, &used);
            if (used != std::string(value).size()) throw std::invalid_argument("Expected an integer");
            return result;
        };
        const int seconds = argc > 2 ? integer(argv[2]) : 10;
        const int domain = argc > 3 ? integer(argv[3]) : 0;
        if (seconds < 1 || seconds > 3600 || domain < 0 || domain > 232)
            throw std::invalid_argument("seconds: 1..3600; domain: 0..232");

        std::mutex mutex;
        TopicTiming sport_timing;
        Sport latest_sport{};
        CloudSummary latest_cloud{};
        unitree::robot::ChannelFactory::Instance()->Init(domain, argv[1]);
        unitree::robot::ChannelSubscriber<Sport> sport_sub("rt/sportmodestate");
        unitree::robot::ChannelSubscriber<Cloud> cloud_sub("rt/utlidar/cloud");
        sport_sub.InitChannel([&](const void* message) {
            const auto now = Clock::now();
            std::lock_guard<std::mutex> guard(mutex);
            sport_timing.observe(now);
            latest_sport = *static_cast<const Sport*>(message);
        }, 1);
        cloud_sub.InitChannel([&](const void* message) {
            const auto now = Clock::now();
            std::lock_guard<std::mutex> guard(mutex);
            TopicTiming timing = latest_cloud.timing;
            timing.observe(now);
            latest_cloud = summarize_cloud(*static_cast<const Cloud*>(message), timing);
        }, 1);

        std::cout << std::fixed << std::setprecision(3)
                  << "SUBSCRIBE ONLY: rt/sportmodestate + rt/utlidar/cloud interface=" << argv[1]
                  << " domain=" << domain << " duration=" << seconds << "s\n"
                  << "No publishers, motor commands, or service clients are created.\n";
        for (int elapsed = 1; elapsed <= seconds; ++elapsed) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            std::lock_guard<std::mutex> guard(mutex);
            const auto now = Clock::now();
            if (sport_timing.count) {
                std::cout << "sport count=" << sport_timing.count << " Hz=" << sport_timing.hz()
                          << " age_ms=" << std::chrono::duration<double, std::milli>(now - sport_timing.last).count()
                          << " max_gap_ms=" << sport_timing.max_gap_ms
                          << " stamp=" << latest_sport.stamp().sec() << '.'
                          << std::setw(9) << std::setfill('0') << latest_sport.stamp().nanosec()
                          << std::setfill(' ') << " mode=" << unsigned(latest_sport.mode())
                          << " error=" << latest_sport.error_code() << " gait=" << unsigned(latest_sport.gait_type())
                          << " position=";
                print_array(latest_sport.position());
                std::cout << " velocity=";
                print_array(latest_sport.velocity());
                std::cout << " body_height=" << latest_sport.body_height()
                          << " yaw_speed=" << latest_sport.yaw_speed() << '\n';
            } else std::cout << "sport WAITING elapsed_s=" << elapsed << '\n';

            if (latest_cloud.timing.count) {
                std::cout << "cloud count=" << latest_cloud.timing.count << " Hz=" << latest_cloud.timing.hz()
                          << " age_ms=" << std::chrono::duration<double, std::milli>(now - latest_cloud.timing.last).count()
                          << " max_gap_ms=" << latest_cloud.timing.max_gap_ms
                          << " stamp=" << latest_cloud.stamp_sec << '.'
                          << std::setw(9) << std::setfill('0') << latest_cloud.stamp_nsec << std::setfill(' ')
                          << " frame='" << latest_cloud.frame_id << "' dims=" << latest_cloud.height << 'x'
                          << latest_cloud.width << " point_step=" << latest_cloud.point_step
                          << " row_step=" << latest_cloud.row_step << " data=" << latest_cloud.data_size
                          << " endian=" << (latest_cloud.big_endian ? "big" : "little")
                          << " dense=" << latest_cloud.dense << " fields=[" << latest_cloud.fields << "]"
                          << " layout=" << (latest_cloud.layout_ok ? "OK" : "MISMATCH")
                          << " xyz_layout=" << (latest_cloud.xyz_layout_ok ? "OK" : "UNSUPPORTED")
                          << " packed_xyz12=" << (latest_cloud.packed_xyz12 ? "YES" : "NO");
                if (latest_cloud.xyz_layout_ok) {
                    std::cout << " finite=" << latest_cloud.finite_points
                              << " nonfinite=" << latest_cloud.nonfinite_points << " xyz_min=";
                    print_array(latest_cloud.xyz_min);
                    std::cout << " xyz_max=";
                    print_array(latest_cloud.xyz_max);
                }
                std::cout << '\n';
            } else std::cout << "cloud WAITING elapsed_s=" << elapsed << '\n';
            std::cout << std::flush;
        }

        sport_sub.CloseChannel();
        cloud_sub.CloseChannel();
        std::lock_guard<std::mutex> guard(mutex);
        const auto now = Clock::now();
        bool failed = false;
        if (!sport_timing.count) { std::cerr << "SPORT NO DATA\n"; failed = true; }
        else if (now - sport_timing.last > std::chrono::seconds(1)) {
            std::cerr << "SPORT STALE: last callback over 1 second old\n"; failed = true;
        }
        if (!latest_cloud.timing.count) { std::cerr << "CLOUD NO DATA\n"; failed = true; }
        else {
            if (now - latest_cloud.timing.last > std::chrono::seconds(1)) {
                std::cerr << "CLOUD STALE: last callback over 1 second old\n"; failed = true;
            }
            if (!latest_cloud.layout_ok || !latest_cloud.xyz_layout_ok) {
                std::cerr << "CLOUD LAYOUT MISMATCH: cannot safely decode XYZ\n"; failed = true;
            }
            // The sidecar decoder now respects PointCloud2 field offsets and strides.
            // Packed XYZ12 is informational; valid hardware XYZ32 is also supported.
        }
        std::cout << "FINAL sport_count=" << sport_timing.count
                  << " cloud_count=" << latest_cloud.timing.count
                  << " verdict=" << (failed ? "CHECK" : "OK") << '\n';
        return failed ? 2 : 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
