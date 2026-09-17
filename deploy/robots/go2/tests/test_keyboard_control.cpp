#include "KeyboardControl.h"
#include "parkour/heading.h"

#include <cassert>
#include <cmath>
#include <iostream>

namespace
{
bool close(float a, float b) { return std::abs(a - b) < 1e-6f; }

void test_axes_and_keys()
{
    Go2KeyboardControl input;
    input.handle_key('i');
    input.handle_key('i');
    input.handle_key('q');
    assert(close(input.axes().speed, .5f));
    assert(close(input.axes().heading_offset, 0.1745329252f));
    input.handle_key('a');
    input.handle_key('d');
    assert(close(input.axes().speed, .5f));
    assert(close(input.axes().heading_offset, 0.1745329252f));
    input.handle_key(' ');
    assert(close(input.axes().speed, 0));
    assert(close(input.axes().turn, 0));
    assert(!input.pending_request());

    for (int i = 0; i < 8; ++i) input.handle_key('e');
    assert(close(input.axes().heading_offset, -8 * 0.1745329252f));
    for (int i = 0; i < 12; ++i) input.handle_key('k');
    assert(close(input.axes().speed, -1));
}

void test_heading_steps()
{
    Go2KeyboardControl input;
    input.set_state(Go2RuntimeState::Policy);
    const float step = 0.1745329252f;
    input.handle_key('q');
    input.handle_key('q');
    input.handle_key('e');
    parkour::HeadingCommand heading;
    for (int i = 0; i < 100; ++i) {
        input.set_heading_keys(false, false); // Idle X11 poll must not erase q/e.
        assert(close(input.axes().heading_offset, step));
        heading.set_relative(i * .05f, input.axes().heading_offset);
        assert(close(heading.delta_yaw(), step * heading.kHeadingScale));
    }
    input.handle_key('e');
    assert(close(input.axes().heading_offset, 0));
    for (int i = 0; i < 19; ++i) input.handle_key('q');
    assert(std::abs(input.axes().heading_offset + 17 * step) < 1e-5f);
    input.handle_key('i');
    input.handle_key('w');
    assert(input.axes().relative_heading && close(input.axes().heading_offset, 0));
    assert(close(input.axes().speed, .25f));
    assert(input.state() == Go2RuntimeState::Policy && !input.pending_request());
    input.handle_key('s'); // Old speed-down binding is inactive.
    assert(close(input.axes().speed, .25f));
    input.handle_key('k');
    assert(close(input.axes().speed, 0));
    input.handle_key('w'); // Repeated reset stays at zero.
    input.set_heading_keys(false, false);
    assert(close(input.axes().heading_offset, 0));
    input.set_heading_keys(true, false);
    assert(close(input.axes().heading_offset, 0.2617993878f));
    input.set_heading_keys(false, false);
    assert(close(input.axes().heading_offset, 0));
    input.handle_key('q');
    input.set_state(Go2RuntimeState::Stand);
    assert(close(input.axes().heading_offset, 0));
}

void test_held_heading()
{
    Go2KeyboardControl input;
    input.set_heading_keys(true, false);
    assert(!input.axes().relative_heading); // Ignore outside Policy.
    input.set_state(Go2RuntimeState::Policy);
    input.handle_key('i');
    input.handle_key('q');
    input.set_heading_keys(true, false);
    auto axes = input.axes();
    const float angle = 0.2617993878f;
    assert(axes.relative_heading && close(axes.heading_offset, angle));
    assert(close(axes.turn, 0) && close(axes.speed, .25f));
    parkour::HeadingCommand heading;
    for (float yaw : {0.0f, 1.0f, 3.1f, -3.1f}) {
        for (int i = 0; i < 100; ++i) {
            input.set_heading_keys(true, false);
            heading.set_relative(yaw, input.axes().heading_offset);
            assert(close(heading.delta_yaw(), angle * heading.kHeadingScale));
        }
        input.set_heading_keys(false, true);
        heading.set_relative(yaw, input.axes().heading_offset);
        assert(close(heading.delta_yaw(), -angle * heading.kHeadingScale));
        input.set_heading_keys(false, false);
        heading.set_relative(yaw, input.axes().heading_offset);
        assert(close(heading.delta_yaw(), 0)); // Release follows new forward.
    }
    input.set_heading_keys(true, true);
    assert(close(input.axes().heading_offset, 0));
    input.set_heading_keys(true, false);
    input.handle_key(' ');
    assert(!input.axes().relative_heading);
    assert(input.consume_transition(Go2RuntimeState::Stand));
    input.set_heading_keys(true, false);
    input.foreground_lost();
    assert(close(input.axes().heading_offset, 0));
    input.set_state(Go2RuntimeState::Stand);
    input.set_state(Go2RuntimeState::Policy);
    assert(!input.axes().relative_heading);
    input.set_heading_keys(true, false);
    input.set_heading_keys(false, false);
    input.handle_key('e');
    assert(input.axes().relative_heading && close(input.axes().heading_offset, -0.1745329252f));
}

void test_requests_and_routing()
{
    const Go2PolicyReadiness ready{true, true, true, true};
    const Go2PolicyReadiness stale{true, false, true, true};
    assert(go2_route_request(Go2RuntimeState::Passive, Go2StateRequest::Stand) == Go2RuntimeState::Stand);
    assert(!go2_route_request(Go2RuntimeState::Passive, Go2StateRequest::Policy, ready));
    assert(!go2_route_request(Go2RuntimeState::Stand, Go2StateRequest::Policy, stale));
    assert(!go2_route_request(Go2RuntimeState::Stand, Go2StateRequest::Policy, {true, true, true, false}));
    assert(go2_route_request(Go2RuntimeState::Stand, Go2StateRequest::Policy, ready) == Go2RuntimeState::Policy);
    assert(go2_route_request(Go2RuntimeState::Policy, Go2StateRequest::Passive) == Go2RuntimeState::Passive);

    Go2KeyboardControl input;
    input.handle_key('1');
    assert(!input.consume_transition(Go2RuntimeState::Passive));
    assert(input.pending_request());
    assert(input.consume_transition(Go2RuntimeState::Stand));
    input.set_state(Go2RuntimeState::Stand);
    input.handle_key('1');
    assert(!input.consume_transition(Go2RuntimeState::Stand));
    assert(!input.pending_request());

    input.handle_key('2');
    assert(!input.consume_transition(Go2RuntimeState::Policy, stale));
    assert(!input.pending_request());
    input.handle_key('2');
    assert(input.consume_request(Go2StateRequest::Policy));
    assert(!input.pending_request());
    input.handle_key('2');
    assert(input.consume_transition(Go2RuntimeState::Policy, ready));

    input.set_state(Go2RuntimeState::Policy);
    input.handle_key('e');
    input.handle_key('i');
    input.set_state(Go2RuntimeState::Stand);
    assert(close(input.axes().speed, 0));
    assert(close(input.axes().turn, 0));
    input.set_state(Go2RuntimeState::Policy);
    input.handle_key('i');
    input.handle_key(' ');
    assert(input.consume_transition(Go2RuntimeState::Stand));
    input.set_state(Go2RuntimeState::Policy);
    input.handle_key('3');
    assert(!input.consume_transition(Go2RuntimeState::StandDown));
    assert(!input.pending_request());
    input.set_state(Go2RuntimeState::Policy);
    input.handle_key('0');
    assert(input.consume_transition(Go2RuntimeState::Passive));
}

void test_down_and_passive()
{
    const Go2PolicyReadiness ready{true, false, true}; // No map needed.
    assert(go2_route_request(Go2RuntimeState::Stand, Go2StateRequest::StandDown, ready) == Go2RuntimeState::StandDown);
    assert(!go2_route_request(Go2RuntimeState::Stand, Go2StateRequest::StandDown));
    assert(!go2_route_request(Go2RuntimeState::Stand, Go2StateRequest::StandDown, {true, true, false}));
    assert(!go2_route_request(Go2RuntimeState::Policy, Go2StateRequest::StandDown, ready));
    assert(!go2_route_request(Go2RuntimeState::Passive, Go2StateRequest::StandDown, ready));
    assert(!go2_route_request(Go2RuntimeState::StandDown, Go2StateRequest::Policy, {true,true,true}));
    assert(go2_route_request(Go2RuntimeState::StandDown, Go2StateRequest::Stand) == Go2RuntimeState::Stand);
    Go2KeyboardControl input;
    input.set_state(Go2RuntimeState::Stand);
    input.handle_key('3');
    assert(!input.consume_transition(Go2RuntimeState::Passive, ready));
    assert(input.consume_transition(Go2RuntimeState::StandDown, ready));
    for (auto state : {Go2RuntimeState::Stand, Go2RuntimeState::StandDown, Go2RuntimeState::Policy}) {
        input.set_state(state);
        input.handle_key('0');
        input.handle_key('3');
        assert(input.pending_request() == Go2StateRequest::Passive);
        assert(input.consume_transition(Go2RuntimeState::Passive));
    }
    assert(!go2_down_joint_settled(-1.602f, -1.5f, .05f, .1f));
    assert(go2_down_joint_settled(-1.602f, -1.5f, .05f, .15f));
    assert(!go2_down_joint_settled(-1.66f, -1.5f, .05f, .15f));
    assert(!go2_down_joint_settled(-1.602f, -1.5f, .21f, .15f));
    assert(!go2_down_joint_settled(NAN, -1.5f, .05f, .15f));
    assert(!go2_down_joint_settled(-1.602f, -1.5f, NAN, .15f));
    assert(!go2_down_joint_settled(-1.602f, -1.5f, .05f, NAN));
    Go2StandDownGate regression;
    for (uint32_t t=0; t<=500; t+=2)
        regression.update(t, go2_down_joint_settled(-1.602f,-1.5f,.05f,.15f));
    assert(regression.ready());
    Go2StandDownGate gate;
    for (uint32_t tick=0; tick<500; tick+=2) { gate.update(tick,true); assert(!gate.ready()); }
    gate.update(500,true); assert(gate.ready());
    gate.update(502,false); assert(!gate.ready());
    gate.update(504,true);
    for (int i=0; i<1000; ++i) gate.update(504,true);
    assert(!gate.ready());  // Repeated stale samples cannot complete dwell.
    gate.update(700,true); assert(!gate.ready());  // Gap resets dwell.
    gate.update(1,true); assert(!gate.ready());  // Clock regression resets.
    gate.reset(); assert(!gate.ready());
    float previous = .8f;
    for (int i=0; i<=300; ++i) {
        float q=go2_pose_smooth(.8f,1.36f,i*.01,3.0);
        assert(q>=previous && q<=1.360001f);
        previous=q;
    }
    assert(close(go2_pose_smooth(.8f,1.36f,0,3),.8f));
    assert(close(go2_pose_smooth(.8f,1.36f,4,3),1.36f));
    assert(close(go2_pose_smooth(.8f,1.36f,.001,3),.8f));
    assert(close(go2_pose_smooth(.8f,1.36f,2.999,3),1.36f));
}

void test_foreground_loss()
{
    Go2KeyboardControl input;
    input.handle_key('1');
    input.foreground_lost();
    assert(!input.pending_request());
    input.set_state(Go2RuntimeState::Policy);
    input.handle_key('i');
    input.foreground_lost();
    assert(close(input.axes().speed, 0));
    assert(input.consume_transition(Go2RuntimeState::Stand));
}

void test_pose_math()
{
    assert(close(go2_pose_lerp(.4f, 1.4f, 0.0), .4f));
    assert(close(go2_pose_lerp(.4f, 1.4f, 1.0), .9f));
    assert(close(go2_pose_lerp(.4f, 1.4f, 2.0), 1.4f));
    assert(close(go2_pose_lerp(.4f, 1.4f, 5.0), 1.4f));
    const std::vector<float> target{0, .8f, -1.5f};
    assert(go2_pose_within_tolerance({.1f, .7f, -1.4f}, target, .11f));
    assert(!go2_pose_within_tolerance({.3f, .8f, -1.5f}, target, .2f));
    assert(!go2_pose_within_tolerance({NAN, .8f, -1.5f}, target, .2f));
    std::vector<float> scan(132, 0.0f);
    assert(go2_scan_values_valid(scan));
    scan[4] = NAN;
    assert(!go2_scan_values_valid(scan));
    scan[4] = 1.01f;
    assert(!go2_scan_values_valid(scan));
    assert(!go2_scan_values_valid(std::vector<float>(131, 0.0f)));
}
}  // namespace

int main()
{
    test_axes_and_keys();
    test_heading_steps();
    test_held_heading();
    test_requests_and_routing();
    test_down_and_passive();
    test_foreground_loss();
    test_pose_math();
    std::cout << "go2 keyboard tests passed\n";
}
