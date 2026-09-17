#include "ShutdownControl.h"
#include <cassert>

struct Fixture {
    Go2ShutdownControl control;
    uint32_t tick = 10;
    uint64_t now = 1'000'000'000;
    Go2ShutdownAction step(Go2RuntimeState state, bool ready = false,
                           bool complete = false, bool down = false,
                           bool fresh = true, bool valid = true, bool upright = true) {
        tick += 10; now += 10'000'000;
        return control.step(state, tick, now, fresh, valid, upright, ready, complete, down);
    }
};
int main()
{
    using State = Go2RuntimeState;
    using Action = Go2ShutdownAction;
    Fixture f;
    assert(f.step(State::Policy) == Action::Hold);
    f.control.request();
    assert(f.step(State::Policy) == Action::Hold); // Need advancing sensor clock.
    assert(f.step(State::Policy) == Action::Stand);
    assert(f.step(State::Stand) == Action::Hold);
    assert(f.step(State::Stand, true) == Action::Down);
    for (int i = 0; i < 400; ++i) f.step(State::StandDown, false, false, true);
    assert(!f.control.complete()); // Measured down does not skip trajectory end.
    for (int i = 0; i < 150; ++i) {
        f.control.request(); // Repeated signals do not reset or force exit.
        f.step(State::StandDown, false, true, true);
        assert(!f.control.complete());
    }
    f.step(State::StandDown, false, true, true);
    assert(f.control.complete());

    Fixture passive;
    passive.control.request();
    for (int i = 0; i < 200; ++i)
        assert(passive.step(State::Passive, false, true, false) == Action::Hold);
    assert(!passive.control.complete()); // Never stand up a Passive robot on exit.
    for (int i = 0; i < 152; ++i) passive.step(State::Passive, false, true, true);
    assert(passive.control.complete());

    Fixture bad;
    bad.control.request();
    for (int i = 0; i < 120; ++i) bad.step(State::StandDown, false, true, true);
    bad.step(State::StandDown, false, true, true, false);
    for (int i = 0; i < 80; ++i) bad.step(State::StandDown, false, true, true);
    assert(!bad.control.complete()); // Freshness loss resets the hold.
    for (int i = 0; i < 200; ++i) {
        bad.now += 10'000'000; // Duplicate tick despite callbacks / wall time.
        bad.control.step(State::StandDown, bad.tick, bad.now, true, true, true, false, true, true);
    }
    assert(!bad.control.complete());
    for (int i = 0; i < 80; ++i) bad.step(State::StandDown, false, true, true);
    assert(!bad.control.complete());
    bad.tick = 1; // Clock reset must break dwell.
    bad.step(State::StandDown, false, true, true);
    for (int i = 0; i < 80; ++i) bad.step(State::StandDown, false, true, true);
    assert(!bad.control.complete());
    for (int i = 0; i < 200; ++i) bad.step(State::StandDown, false, true, true, true, false);
    assert(!bad.control.complete());
    for (int i = 0; i < 200; ++i) bad.step(State::StandDown, false, true, true, true, true, false);
    assert(!bad.control.complete());
    assert(bad.step(State::Policy, false, false, false, true, true, false) == Action::Stand);
    assert(bad.step(State::Stand, true, false, false, false) == Action::Hold);
}
