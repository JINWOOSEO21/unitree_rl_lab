// Subscriber-only bounded DDS recorder for inspecting Go2 LiDAR frame products.
// It creates no publishers, clients, or service calls.
#define main go2_record_original_main
#include "go2_record.cpp"
#undef main

#include <unitree/idl/ros2/Odometry_.hpp>
#include <unitree/idl/ros2/PoseStamped_.hpp>

#include <cstdlib>
#include <memory>
#include <vector>

namespace {
using Odom = nav_msgs::msg::dds_::Odometry_;
using PoseStamped = geometry_msgs::msg::dds_::PoseStamped_;
constexpr uint64_t kMaxBytesPerCloud = 128ULL * 1024ULL * 1024ULL;

void point_json(std::ostream& out, const geometry_msgs::msg::dds_::Point_& p)
{
    out << "{\"x\":" << p.x() << ",\"y\":" << p.y() << ",\"z\":" << p.z() << '}';
}

void quaternion_json(std::ostream& out, const geometry_msgs::msg::dds_::Quaternion_& q)
{
    out << "{\"x\":" << q.x() << ",\"y\":" << q.y() << ",\"z\":" << q.z()
        << ",\"w\":" << q.w() << '}';
}

void vector_json(std::ostream& out, const geometry_msgs::msg::dds_::Vector3_& v)
{
    out << "{\"x\":" << v.x() << ",\"y\":" << v.y() << ",\"z\":" << v.z() << '}';
}

void pose_json(std::ostream& out, uint64_t time_ns, const PoseStamped& message)
{
    out << std::setprecision(17) << "{\"steady_ns\":" << time_ns
        << ",\"stamp\":{\"sec\":" << message.header().stamp().sec()
        << ",\"nanosec\":" << message.header().stamp().nanosec() << "},\"frame_id\":";
    string_json(out, message.header().frame_id());
    out << ",\"position\":";
    point_json(out, message.pose().position());
    out << ",\"orientation\":";
    quaternion_json(out, message.pose().orientation());
    out << "}\n";
}

void odom_json(std::ostream& out, uint64_t time_ns, const Odom& message)
{
    const auto& pose = message.pose().pose();
    const auto& twist = message.twist().twist();
    out << std::setprecision(17) << "{\"steady_ns\":" << time_ns
        << ",\"stamp\":{\"sec\":" << message.header().stamp().sec()
        << ",\"nanosec\":" << message.header().stamp().nanosec() << "},\"frame_id\":";
    string_json(out, message.header().frame_id());
    out << ",\"child_frame_id\":";
    string_json(out, message.child_frame_id());
    out << ",\"position\":";
    point_json(out, pose.position());
    out << ",\"orientation\":";
    quaternion_json(out, pose.orientation());
    out << ",\"linear_velocity\":";
    vector_json(out, twist.linear());
    out << ",\"angular_velocity\":";
    vector_json(out, twist.angular());
    out << ",\"pose_covariance\":";
    array_json(out, message.pose().covariance());
    out << ",\"twist_covariance\":";
    array_json(out, message.twist().covariance());
    out << "}\n";
}

struct CloudSink {
    explicit CloudSink(const std::filesystem::path& root, std::string topic_name)
        : topic(std::move(topic_name))
    {
        std::filesystem::create_directories(root);
        meta.open(root / "cloud.jsonl");
        binary.open(root / "cloud.bin", std::ios::binary);
        if (!meta || !binary) throw std::runtime_error("failed to open cloud output: " + root.string());
    }

    void write(uint64_t time_ns, const Cloud& cloud)
    {
        std::lock_guard<std::mutex> lock(mutex);
        const uint64_t size = cloud.data().size();
        const uint64_t offset = bytes.load();
        if (size > kMaxBytesPerCloud - offset) {
            limit_hit = true;
            return;
        }
        binary.write(reinterpret_cast<const char*>(cloud.data().data()), size);
        cloud_json(meta, time_ns, cloud, offset, size);
        bytes += size;
        ++count;
        if (first_frame.empty()) first_frame = cloud.header().frame_id();
        last_frame = cloud.header().frame_id();
    }

    void close()
    {
        std::lock_guard<std::mutex> lock(mutex);
        meta.flush();
        binary.flush();
        if (!meta || !binary) throw std::runtime_error("cloud write failure for " + topic);
    }

    std::string topic;
    std::ofstream meta;
    std::ofstream binary;
    std::mutex mutex;
    std::atomic<uint64_t> count{0};
    std::atomic<uint64_t> bytes{0};
    std::atomic<bool> limit_hit{false};
    std::string first_frame;
    std::string last_frame;
};
}  // namespace

int main(int argc, char** argv)
{
    if (argc < 3 || argc > 5 || std::string(argv[1]) == "--help") {
        std::cout << "Usage: go2_frame_record <interface> <output_dir> [seconds=8] [domain=0]\n"
                  << "SUBSCRIBER ONLY: records three LiDAR clouds, low/sport state, odom and pose.\n";
        return argc == 2 ? 0 : 1;
    }
    try {
        const auto integer = [](const char* value) {
            size_t used = 0;
            const int result = std::stoi(value, &used);
            if (used != std::string(value).size()) throw std::invalid_argument("Expected integer");
            return result;
        };
        const int seconds = argc > 3 ? integer(argv[3]) : 8;
        const int domain = argc > 4 ? integer(argv[4]) : 0;
        if (seconds < 1 || seconds > 30 || domain < 0 || domain > 232)
            throw std::invalid_argument("seconds: 1..30; domain: 0..232");

        const std::filesystem::path output(argv[2]);
        if (std::filesystem::exists(output)) throw std::runtime_error("output_dir already exists");
        std::filesystem::create_directories(output);
        std::ofstream low_file(output / "lowstate.jsonl");
        std::ofstream sport_file(output / "sportmodestate.jsonl");
        std::ofstream odom_file(output / "robot_odom.jsonl");
        std::ofstream pose_file(output / "robot_pose.jsonl");
        if (!low_file || !sport_file || !odom_file || !pose_file)
            throw std::runtime_error("failed to open state output files");

        CloudSink raw(output / "utlidar_cloud", "rt/utlidar/cloud");
        CloudSink base(output / "utlidar_cloud_base", "rt/utlidar/cloud_base");
        CloudSink deskewed(output / "utlidar_cloud_deskewed", "rt/utlidar/cloud_deskewed");
        const auto start = Clock::now();
        std::mutex low_mutex, sport_mutex, odom_mutex, pose_mutex;
        std::atomic<uint64_t> low_count{0}, sport_count{0}, odom_count{0}, pose_count{0};
        std::atomic<bool> accepting{true};

        unitree::robot::ChannelFactory::Instance()->Init(domain, argv[1]);
        unitree::robot::ChannelSubscriber<Low> low_sub("rt/lowstate");
        unitree::robot::ChannelSubscriber<Sport> sport_sub("rt/sportmodestate");
        unitree::robot::ChannelSubscriber<Cloud> raw_sub("rt/utlidar/cloud");
        unitree::robot::ChannelSubscriber<Cloud> base_sub("rt/utlidar/cloud_base");
        unitree::robot::ChannelSubscriber<Cloud> deskewed_sub("rt/utlidar/cloud_deskewed");
        unitree::robot::ChannelSubscriber<Odom> odom_sub("rt/utlidar/robot_odom");
        unitree::robot::ChannelSubscriber<PoseStamped> pose_sub("rt/utlidar/robot_pose");

        low_sub.InitChannel([&](const void* data) {
            if (!accepting.load(std::memory_order_acquire)) return;
            const auto now = steady_ns(start);
            std::lock_guard<std::mutex> lock(low_mutex);
            low_json(low_file, now, *static_cast<const Low*>(data));
            ++low_count;
        }, 1);
        sport_sub.InitChannel([&](const void* data) {
            if (!accepting.load(std::memory_order_acquire)) return;
            const auto now = steady_ns(start);
            std::lock_guard<std::mutex> lock(sport_mutex);
            sport_json(sport_file, now, *static_cast<const Sport*>(data));
            ++sport_count;
        }, 1);
        raw_sub.InitChannel([&](const void* data) { if (accepting.load(std::memory_order_acquire)) raw.write(steady_ns(start), *static_cast<const Cloud*>(data)); }, 1);
        base_sub.InitChannel([&](const void* data) { if (accepting.load(std::memory_order_acquire)) base.write(steady_ns(start), *static_cast<const Cloud*>(data)); }, 1);
        deskewed_sub.InitChannel([&](const void* data) { if (accepting.load(std::memory_order_acquire)) deskewed.write(steady_ns(start), *static_cast<const Cloud*>(data)); }, 1);
        odom_sub.InitChannel([&](const void* data) {
            if (!accepting.load(std::memory_order_acquire)) return;
            const auto now = steady_ns(start);
            std::lock_guard<std::mutex> lock(odom_mutex);
            odom_json(odom_file, now, *static_cast<const Odom*>(data));
            ++odom_count;
        }, 1);
        pose_sub.InitChannel([&](const void* data) {
            if (!accepting.load(std::memory_order_acquire)) return;
            const auto now = steady_ns(start);
            std::lock_guard<std::mutex> lock(pose_mutex);
            pose_json(pose_file, now, *static_cast<const PoseStamped*>(data));
            ++pose_count;
        }, 1);

        std::cout << "SUBSCRIBE ONLY interface=" << argv[1] << " domain=" << domain
                  << " duration=" << seconds << "s output=" << output << '\n';
        std::this_thread::sleep_for(std::chrono::seconds(seconds));
        accepting.store(false, std::memory_order_release);
        std::this_thread::sleep_for(std::chrono::milliseconds(250));

        { std::lock_guard<std::mutex> lock(low_mutex); low_file.flush(); }
        { std::lock_guard<std::mutex> lock(sport_mutex); sport_file.flush(); }
        { std::lock_guard<std::mutex> lock(odom_mutex); odom_file.flush(); }
        { std::lock_guard<std::mutex> lock(pose_mutex); pose_file.flush(); }
        raw.close(); base.close(); deskewed.close();

        std::ofstream manifest(output / "manifest.json");
        manifest << "{\"format_version\":1,\"subscriber_only\":true,\"interface\":";
        string_json(manifest, argv[1]);
        manifest << ",\"domain\":" << domain << ",\"requested_duration_s\":" << seconds
                 << ",\"timestamp_semantics\":\"steady nanoseconds since recorder start, sampled at callback entry\""
                 << ",\"max_bytes_per_cloud\":" << kMaxBytesPerCloud
                 << ",\"lowstate_count\":" << low_count.load() << ",\"sportmodestate_count\":" << sport_count.load()
                 << ",\"robot_odom_count\":" << odom_count.load() << ",\"robot_pose_count\":" << pose_count.load()
                 << ",\"clouds\":[";
        const std::array<CloudSink*, 3> clouds{&raw, &base, &deskewed};
        for (size_t i = 0; i < clouds.size(); ++i) {
            const auto& sink = *clouds[i];
            if (i) manifest << ',';
            manifest << "{\"topic\":"; string_json(manifest, sink.topic);
            manifest << ",\"count\":" << sink.count.load() << ",\"bytes\":" << sink.bytes.load()
                     << ",\"limit_hit\":" << (sink.limit_hit.load() ? "true" : "false")
                     << ",\"first_frame_id\":"; string_json(manifest, sink.first_frame);
            manifest << ",\"last_frame_id\":"; string_json(manifest, sink.last_frame);
            manifest << '}';
        }
        manifest << "]}\n";
        manifest.close();

        std::cout << "counts low=" << low_count.load() << " sport=" << sport_count.load()
                  << " odom=" << odom_count.load() << " pose=" << pose_count.load() << '\n';
        for (const auto* sink : clouds)
            std::cout << sink->topic << " count=" << sink->count.load() << " bytes=" << sink->bytes.load()
                      << " frame=" << sink->first_frame << '\n';
        if (!low_file || !sport_file || !odom_file || !pose_file || !manifest)
            throw std::runtime_error("write failure");
        const int result = (raw.limit_hit || base.limit_hit || deskewed.limit_hit) ? 3 : 0;
        std::cout.flush();
        std::cerr.flush();
        // This installed SDK2/Cyclone build asserts while explicitly closing several
        // subscribers. All callbacks are quiesced and files flushed above; bypass
        // subscriber destructors so a completed diagnostic capture exits cleanly.
        std::_Exit(result);
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
