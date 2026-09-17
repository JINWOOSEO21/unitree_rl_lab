#pragma once

#include "FSM/State_Passive.h"
#include "KeyboardControl.h"
#include "Go2Shutdown.h"

class State_Go2Passive : public State_Passive
{
public:
    State_Go2Passive(int state, std::string state_string) : State_Passive(state, state_string)
    {
        go2_guard_operator_routes(*this);
        go2_add_shutdown_routes(*this, Go2RuntimeState::Passive);
        add_keyboard_routes();
    }

    void enter() override
    {
        if (go2_keyboard_control) go2_keyboard_control->set_state(Go2RuntimeState::Passive);
        State_Passive::enter();
    }

private:
    void add_keyboard_routes()
    {
        auto check = [](Go2RuntimeState target) {
            return [target] { return go2_keyboard_control && go2_keyboard_control->consume_transition(target); };
        };
        registered_checks.emplace_back(check(Go2RuntimeState::Passive), FSMStringMap.right.at("Passive"));
        registered_checks.emplace_back(check(Go2RuntimeState::Stand), FSMStringMap.right.at("FixStand"));
        registered_checks.emplace_back(check(Go2RuntimeState::Policy), FSMStringMap.right.at("Parkour"));
    }
};

REGISTER_FSM(State_Go2Passive)
