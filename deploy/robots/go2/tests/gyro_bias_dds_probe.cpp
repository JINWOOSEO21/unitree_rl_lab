// Integration probe: loopback only, no robot commands or LowState subscriptions.
#include "GyroBiasSubscriber.h"
#include <unitree/robot/channel/channel_factory.hpp>
#include <iostream>
#include <thread>
int main() {
    unitree::robot::ChannelFactory::Instance()->Init(181, "lo");
    Go2GyroBiasSubscriber subscriber;
    bool first=false, invalidated=false, second=false, final_invalid=false;
    parkour::GyroBiasSample latched;
    std::cout << "READY loopback domain 181" << std::endl;
    for (int i=0; i<1000; ++i) {
        parkour::GyroBiasSample current;
        bool valid=subscriber.get(current);
        if (valid && std::abs(current.bias[0]-.01f)<1e-6f) { first=true; latched=current; }
        if (first && !valid) invalidated=true;
        if (invalidated && valid && std::abs(current.bias[0]-.02f)<1e-6f && current.session!=latched.session) second=true;
        if (second && !valid) { final_invalid=true; break; }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    bool ok=first && invalidated && second && final_invalid && std::abs(latched.bias[0]-.01f)<1e-6f;
    std::cout << "first=" << first << " restart_invalid=" << invalidated << " second=" << second
              << " final_invalid=" << final_invalid << " frozen=" << latched.bias[0] << std::endl;
    return ok ? 0 : 1;
}
