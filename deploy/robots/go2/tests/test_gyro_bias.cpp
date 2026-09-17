#include "parkour/gyro_bias.h"
#include "parkour/observation.h"
#include <cassert>
#include <sstream>
std::string message(std::string session, int seq, int tick, bool calibrated, float x=.01f) {
    std::ostringstream s;
    s << "{\"version\":1,\"session\":\"" << session << "\",\"sequence\":" << seq
      << ",\"source_tick\":" << tick << ",\"calibrated\":" << (calibrated ? "true" : "false")
      << ",\"frame_id\":\"base_link\",\"units\":\"rad/s\",\"bias_rad_s\":[" << x << ",0.02,-0.03]}";
    return s.str();
}
int main() {
    parkour::GyroBiasCache cache;
    parkour::GyroBiasSample sample;
    assert(!cache.get(0,sample));
    cache.accept(message("first",1,0,false),0);
    assert(!cache.get(0,sample));
    cache.accept(message("first",2,100,true),.2);
    assert(cache.get(.2,sample));
    const auto applied = sample; // Controller latches a value copy at entry.
    cache.accept(message("first",3,200,true,.04),.4);
    assert(cache.get(.4,sample) && sample.bias[0] == .04f);
    assert(applied.bias[0] == .01f);
    cache.accept(message("first",3,200,true),1.2); // Duplicate cannot refresh.
    assert(!cache.get(1.5,sample));
    cache.accept(message("second",1,0,false),1.6);
    assert(!cache.get(1.6,sample));
    cache.accept(message("first",4,300,true),1.7);
    assert(!cache.get(1.7,sample)); // Old delayed source cannot reactivate.
    cache.accept(message("second",2,400,true),1.8);
    assert(cache.get(1.8,sample) && sample.session == "second");
    assert(applied.session == "first");
    cache.accept(message("second",3,400,true),2.0);
    assert(!cache.get(2.0,sample)); // Frozen LowState tick despite heartbeat.
    cache.accept(message("second",4,500,true),2.2);
    assert(cache.get(2.2,sample));
    cache.accept("garbage",2.3);
    assert(!cache.get(2.3,sample));
    cache.accept(message("second",5,600,true,1.0),2.4);
    assert(!cache.get(2.4,sample));
    parkour::GyroBiasCache restarted_controller;
    restarted_controller.accept(message("second",6,700,true),3);
    assert(restarted_controller.get(3,sample)); // Late subscriber gets heartbeat.
    auto malformed=message("second",7,800,true);
    malformed.replace(malformed.find("rad/s"),5,"deg/s");
    restarted_controller.accept(malformed,3.1);
    assert(!restarted_controller.get(3.1,sample));
    parkour::Contract contract;
    contract.default_joint_pos.assign(12, 0.0f);
    parkour::ObservationBuilder obs(contract);
    parkour::PropInputs in;
    in.gyro_b = Eigen::Vector3f(.11f-applied.bias[0], .22f-applied.bias[1], .27f-applied.bias[2]);
    auto prop=obs.build_prop(in);
    assert(std::abs(prop[0]-.025f)<1e-6 && std::abs(prop[1]-.05f)<1e-6 && std::abs(prop[2]-.075f)<1e-6);
    obs.prime(prop);
    assert(std::abs(obs.history()[0]-.025f)<1e-6);
}
