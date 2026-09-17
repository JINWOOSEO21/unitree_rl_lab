#pragma once
#include <chrono>
#include <memory>
#include <unitree/idl/ros2/String_.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include "parkour/gyro_bias.h"

class Go2GyroBiasSubscriber {
public:
    Go2GyroBiasSubscriber() {
        sub_ = std::make_unique<unitree::robot::ChannelSubscriber<std_msgs::msg::dds_::String_>>("rt/parkour/gyro_bias");
        sub_->InitChannel([this](const void* msg) {
            cache_.accept(static_cast<const std_msgs::msg::dds_::String_*>(msg)->data(), now());
        }, 1);
    }
    ~Go2GyroBiasSubscriber() { sub_->CloseChannel(); }
    bool get(parkour::GyroBiasSample& out, std::string* reason = nullptr) const {
        return cache_.get(now(), out, reason);
    }
private:
    static double now() {
        return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
    }
    parkour::GyroBiasCache cache_;
    std::unique_ptr<unitree::robot::ChannelSubscriber<std_msgs::msg::dds_::String_>> sub_;
};
inline std::unique_ptr<Go2GyroBiasSubscriber> go2_gyro_bias;
