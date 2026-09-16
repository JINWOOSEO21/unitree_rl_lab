#include "FSM/CtrlFSM.h"
#include "FSM/State_Go2Passive.h"
#include "FSM/State_Go2Pose.h"
#include "FSM/State_RLBase.h"
#include "FSM/State_Parkour.h"
#include "KeyboardControl.h"
#include "TerminalInput.h"

#include <atomic>
#include <csignal>
#include <cstdlib>

namespace
{
std::atomic<bool> keep_running{true};
void stop_on_signal(int) { keep_running = false; }

struct LocalOptions
{
    bool keyboard = false;
    bool keyboard_check = false;
    std::vector<std::string> forwarded;
};

LocalOptions parse_local_options(int argc, char** argv)
{
    LocalOptions result;
    result.forwarded.emplace_back(argv[0]);
    for (int i = 1; i < argc; ++i) {
        const std::string argument(argv[i]);
        if (argument == "--keyboard") result.keyboard = true;
        else if (argument == "--keyboard-check") {
            result.keyboard = true;
            result.keyboard_check = true;
        } else result.forwarded.push_back(argument);
    }
    return result;
}

bool terminal_is_foreground()
{
    return isatty(STDIN_FILENO) && tcgetpgrp(STDIN_FILENO) == getpgrp();
}
}  // namespace

std::unique_ptr<LowCmd_t> FSMState::lowcmd = nullptr;
std::shared_ptr<LowState_t> FSMState::lowstate = nullptr;
std::shared_ptr<Keyboard> FSMState::keyboard = nullptr;

void init_fsm_state()
{
    auto lowcmd_sub = std::make_shared<unitree::robot::go2::subscription::LowCmd>();
    usleep(0.2 * 1e6);
    if(!lowcmd_sub->isTimeout())
    {
        spdlog::critical("The other process is using the lowcmd channel, please close it first.");
        unitree::robot::go2::shutdown();
        // exit(0);
    }
    FSMState::lowcmd = std::make_unique<LowCmd_t>();
    FSMState::lowstate = std::make_shared<LowState_t>();
    spdlog::info("Waiting for connection to robot...");
    FSMState::lowstate->wait_for_connection();
    spdlog::info("Connected to robot.");
}

int main(int argc, char** argv)
{
    const auto options = parse_local_options(argc, argv);
    if (options.keyboard && !terminal_is_foreground()) {
        std::cerr << "--keyboard requires an interactive foreground terminal; DDS was not initialized.\n";
        return 2;
    }
    std::vector<char*> forwarded;
    forwarded.reserve(options.forwarded.size());
    for (const auto& argument : options.forwarded) forwarded.push_back(const_cast<char*>(argument.c_str()));

    if (options.keyboard_check) {
        Go2KeyboardControl check_control;
        Go2TerminalInput terminal(check_control);
        std::cout << "[keyboard-check] NO DDS / NO MOTOR OUTPUT. Input echo only.\n";
        Go2TerminalInput::print_help();
        std::signal(SIGINT, stop_on_signal);
        std::signal(SIGTERM, stop_on_signal);
        while (keep_running.load()) {
            terminal.poll();
            usleep(10000);
        }
        return 0;
    }

    // Load parameters
    auto vm = param::helper(static_cast<int>(forwarded.size()), forwarded.data());

    std::cout << " --- Unitree Robotics --- \n";
    std::cout << "     Go2 Controller \n";

    std::unique_ptr<Go2TerminalInput> terminal;
    if (options.keyboard) {
        go2_keyboard_control = std::make_shared<Go2KeyboardControl>();
        terminal = std::make_unique<Go2TerminalInput>(*go2_keyboard_control);
        Go2TerminalInput::print_help();
        std::signal(SIGINT, stop_on_signal);
        std::signal(SIGTERM, stop_on_signal);
    }

    // Unitree DDS Config
    unitree::robot::ChannelFactory::Instance()->Init(0, vm["network"].as<std::string>());

    init_fsm_state();

    // Initialize FSM
    auto fsm = std::make_unique<CtrlFSM>(param::config["FSM"]);
    fsm->start();

    if (!options.keyboard) {
        std::cout << "Press [L2 + A] to enter FixStand mode.\n";
        std::cout << "And then press [Start] to start controlling the robot.\n";
    }

    while (keep_running.load()) {
        if (terminal) terminal->poll();
        usleep(10000);
    }
    // CtrlFSM has no stop API. Restore the terminal, then terminate the process without
    // racing its recurrent control thread against state destruction.
    terminal.reset();
    std::_Exit(130);
}
