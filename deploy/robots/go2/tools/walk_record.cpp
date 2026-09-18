// Walk telemetry recorder. Subscriber only: no publisher, no LowCmd, no service client.
// Usage: go2_walk_record <interface> <seconds> <out_base> [domain=0]
//
// Step E-3 asks what happens while the robot actually walks, and the bridge cannot answer it.
// Running the bridge without --summary-only would make it json.dumps every LowState sample at
// 500 Hz on the Jetson, which is the same kind of load that starved the cloud reader and the
// tick loop before (see migration/README.md). So record from here instead: the laptop has the
// headroom, and nothing this process does can perturb the control path.
//
// rt/parkour/scandots carries the leg odometry origin (ScandotsOutput publishes
// origin=position[:2]), so the odometry trajectory is already on the wire at 10 Hz -- no extra
// bridge output is needed to see drift.
//
// Writes two TSVs:
//   <base>.low.tsv   t_ms tick ff0..ff3 qw qx qy qz gx gy gz ax ay az     (500 Hz)
//   <base>.scan.tsv  t_ms stamp_s ox oy n                                 (10 Hz)
//
// DDS callbacks only append to a buffer; the main loop does the file writes, so a slow disk
// can never stall a reader thread.

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/HeightMap_.hpp>

#include <chrono>
#include <csignal>
#include <cstdio>
#include <fstream>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace
{
volatile std::sig_atomic_t keep_running = 1;
void stop_on_signal(int) { keep_running = 0; }

struct LowRow
{
    double t_ms;
    uint32_t tick;
    int16_t foot[4];
    float quat[4];
    float gyro[3];
    float acc[3];
};

struct ScanRow
{
    double t_ms;
    double stamp_s;
    double origin[2];
    int n;
};

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
    if (argc < 4 || argc > 5 || std::string(argv[1]) == "--help") {
        std::puts("Usage: go2_walk_record <interface> <seconds> <out_base> [domain=0]\n"
                  "Subscribes to rt/lowstate and rt/parkour/scandots only.\n"
                  "No publisher is created and nothing is sent to the robot.\n"
                  "Writes <out_base>.low.tsv and <out_base>.scan.tsv.");
        return argc == 2 ? 0 : 1;
    }
    try {
        const int seconds = as_int(argv[2], "seconds", 1, 3600);
        const std::string base = argv[3];
        const int domain = argc > 4 ? as_int(argv[4], "domain", 0, 232) : 0;

        using Clock = std::chrono::steady_clock;
        const auto start = Clock::now();
        const auto elapsed_ms = [&start] {
            return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
        };

        std::mutex mutex;
        std::vector<LowRow> low_buffer;
        std::vector<ScanRow> scan_buffer;
        uint64_t low_total = 0, scan_total = 0, scan_invalidations = 0;

        unitree::robot::ChannelFactory::Instance()->Init(domain, argv[1]);

        unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::LowState_> low_sub("rt/lowstate");
        low_sub.InitChannel([&](const void* message) {
            const auto* m = static_cast<const unitree_go::msg::dds_::LowState_*>(message);
            LowRow row{};
            row.t_ms = elapsed_ms();
            row.tick = m->tick();
            for (int i = 0; i < 4; ++i) row.foot[i] = m->foot_force()[i];
            for (int i = 0; i < 4; ++i) row.quat[i] = m->imu_state().quaternion()[i];
            for (int i = 0; i < 3; ++i) row.gyro[i] = m->imu_state().gyroscope()[i];
            for (int i = 0; i < 3; ++i) row.acc[i] = m->imu_state().accelerometer()[i];
            std::lock_guard<std::mutex> guard(mutex);
            low_buffer.push_back(row);
            ++low_total;
        }, 1);

        unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::HeightMap_>
            scan_sub("rt/parkour/scandots");
        scan_sub.InitChannel([&](const void* message) {
            const auto* m = static_cast<const unitree_go::msg::dds_::HeightMap_*>(message);
            ScanRow row{};
            row.t_ms = elapsed_ms();
            row.stamp_s = m->stamp();
            row.origin[0] = m->origin()[0];
            row.origin[1] = m->origin()[1];
            row.n = static_cast<int>(m->data().size());
            std::lock_guard<std::mutex> guard(mutex);
            scan_buffer.push_back(row);
            ++scan_total;
            // An empty map is the bridge explicitly invalidating the previous scan.
            if (row.n == 0) ++scan_invalidations;
        }, 1);

        std::ofstream low_file(base + ".low.tsv");
        std::ofstream scan_file(base + ".scan.tsv");
        if (!low_file || !scan_file) throw std::runtime_error("cannot open output files");
        low_file << "t_ms\ttick\tff0\tff1\tff2\tff3\tqw\tqx\tqy\tqz\tgx\tgy\tgz\tax\tay\taz\n";
        scan_file << "t_ms\tstamp_s\tox\toy\tn\n";

        std::printf("SUBSCRIBE ONLY: rt/lowstate + rt/parkour/scandots  interface=%s domain=%d "
                    "duration=%ds\n  -> %s.low.tsv  %s.scan.tsv\n",
                    argv[1], domain, seconds, base.c_str(), base.c_str());
        std::fflush(stdout);

        std::signal(SIGINT, stop_on_signal);
        std::signal(SIGTERM, stop_on_signal);

        uint64_t low_seen = 0, scan_seen = 0;
        for (int second = 0; second < seconds && keep_running; ++second) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            std::vector<LowRow> low_batch;
            std::vector<ScanRow> scan_batch;
            uint64_t bad = 0;
            {
                std::lock_guard<std::mutex> guard(mutex);
                low_batch.swap(low_buffer);
                scan_batch.swap(scan_buffer);
                bad = scan_invalidations;
            }
            for (const auto& r : low_batch) {
                low_file << r.t_ms << '\t' << r.tick;
                for (int i = 0; i < 4; ++i) low_file << '\t' << r.foot[i];
                for (int i = 0; i < 4; ++i) low_file << '\t' << r.quat[i];
                for (int i = 0; i < 3; ++i) low_file << '\t' << r.gyro[i];
                for (int i = 0; i < 3; ++i) low_file << '\t' << r.acc[i];
                low_file << '\n';
            }
            for (const auto& r : scan_batch)
                scan_file << r.t_ms << '\t' << r.stamp_s << '\t' << r.origin[0] << '\t'
                          << r.origin[1] << '\t' << r.n << '\n';
            low_file.flush();
            scan_file.flush();
            low_seen += low_batch.size();
            scan_seen += scan_batch.size();
            std::printf("t=%4ds  low=%llu(+%zu)  scan=%llu(+%zu)  invalidated=%llu\n", second + 1,
                        static_cast<unsigned long long>(low_seen), low_batch.size(),
                        static_cast<unsigned long long>(scan_seen), scan_batch.size(),
                        static_cast<unsigned long long>(bad));
            std::fflush(stdout);
        }

        low_sub.CloseChannel();
        scan_sub.CloseChannel();
        std::printf("done: low=%llu scan=%llu\n", static_cast<unsigned long long>(low_total),
                    static_cast<unsigned long long>(scan_total));
        return low_total == 0 ? 2 : 0;
    } catch (const std::exception& error) {
        std::fprintf(stderr, "%s\n", error.what());
        return 1;
    }
}
