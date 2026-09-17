#pragma once
#include <array>
#include <cmath>
#include <cstdint>
#include <mutex>
#include <string>
#include <unordered_set>
#include <yaml-cpp/yaml.h>

namespace parkour {
struct GyroBiasSample {
    std::array<float, 3> bias{};
    std::string session;
    uint64_t sequence = 0, tick = 0;
};

// One source session at a time. Reception time is the controller's steady clock.
class GyroBiasCache {
public:
    void accept(const std::string& payload, double now) {
        std::lock_guard<std::mutex> lock(mutex_);
        try {
            if (payload.size() > 4096) throw std::runtime_error("oversize gyro bias message");
            const auto n = YAML::Load(payload);
            if (n["version"].as<int>() != 1 || n["frame_id"].as<std::string>() != "base_link" ||
                n["units"].as<std::string>() != "rad/s") throw std::runtime_error("gyro bias schema/frame mismatch");
            GyroBiasSample sample;
            sample.session = n["session"].as<std::string>();
            if (sample.session.empty() || sample.session.size() > 128) throw std::runtime_error("invalid gyro bias session");
            const auto sequence = n["sequence"].as<int64_t>(), tick = n["source_tick"].as<int64_t>();
            if (sequence <= 0 || tick < 0) throw std::runtime_error("invalid gyro bias sequence/tick");
            sample.sequence = sequence; sample.tick = tick;
            const auto values = n["bias_rad_s"];
            if (!values.IsSequence() || values.size() != 3) throw std::runtime_error("gyro bias needs 3 values");
            for (size_t i = 0; i < 3; ++i) {
                sample.bias[i] = values[i].as<float>();
                if (!std::isfinite(sample.bias[i]) || std::abs(sample.bias[i]) > 0.1f)
                    throw std::runtime_error("gyro bias outside calibration bounds");
            }
            const bool calibrated = n["calibrated"].as<bool>();
            if (retired_.count(sample.session)) return; // Delayed old session cannot reactivate.
            if (sample.session != latest_.session) {
                if (!latest_.session.empty()) retired_.insert(latest_.session);
                valid_ = false;
                latest_ = {};
            } else if (sample.sequence <= latest_.sequence) {
                return; // Duplicate/out-of-order heartbeat cannot refresh age.
            } else if (calibrated && sample.tick <= latest_.tick) {
                valid_ = false;
                reason_ = "gyro bias LowState tick did not advance";
                latest_.sequence = sample.sequence;
                return;
            }
            latest_ = sample;
            valid_ = calibrated;
            received_ = now;
            reason_ = calibrated ? "" : "bridge gyro calibration incomplete or LowState stale";
        } catch (const std::exception& e) {
            valid_ = false;
            reason_ = e.what();
        }
    }
    bool get(double now, GyroBiasSample& out, std::string* reason = nullptr) const {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!valid_ || now < received_ || now - received_ > 1.0) {
            if (reason) *reason = valid_ ? "gyro bias heartbeat expired (>1 s)" : reason_;
            return false;
        }
        out = latest_;
        return true;
    }
private:
    mutable std::mutex mutex_;
    GyroBiasSample latest_;
    bool valid_ = false;
    double received_ = 0;
    std::string reason_ = "no calibrated rt/parkour/gyro_bias received";
    std::unordered_set<std::string> retired_;
};
} // namespace parkour
