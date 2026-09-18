#include "FSM/CtrlFSM.h"
#include "FSM/State_Go2Passive.h"
#include "FSM/State_Go2Pose.h"
#include "FSM/State_RLBase.h"
#include "FSM/State_Parkour.h"
#include "KeyboardControl.h"
#include "StartupHandoff.h"
#include "Go2Shutdown.h"
#include "TerminalInput.h"

#include <algorithm>
#include <atomic>
#include <csignal>
#include <cstdlib>
#include <stdexcept>

#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>
#include <unitree/robot/go2/sport/sport_client.hpp>

namespace
{
volatile std::sig_atomic_t keep_running = 1;
volatile std::sig_atomic_t shutdown_signal = SIGINT;
void stop_on_signal(int signal) { shutdown_signal = signal; keep_running = 0; }

struct LocalOptions
{
    bool keyboard = false;
    bool keyboard_check = false;
    bool simulator = false;
    std::vector<std::string> forwarded;
};

LocalOptions parse_local_options(int argc, char** argv)
{
    LocalOptions result;
    result.forwarded.emplace_back(argv[0]);
    for (int i = 1; i < argc; ++i) {
        const std::string argument(argv[i]);
        if (argument == "--keyboard") result.keyboard = true;
        else if (argument == "--sim") result.simulator = true;
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

uint64_t steady_now_ns()
{
    const auto now = std::chrono::steady_clock::now().time_since_epoch();
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(now).count());
}

class StartupLowState : public LowState_t
{
public:
    ~StartupLowState() { sub_->CloseChannel(); }
    uint64_t receipt_ns() const { return receipt_ns_.load(); }

protected:
    void post_communication() override
    {
        const uint32_t tick = msg_.tick();
        if (clock_invalid_.load()) return;
        const bool had_tick = have_tick_.load();
        const int32_t delta = static_cast<int32_t>(tick - last_tick_.load());
        const bool advances = !had_tick || delta > 0;
        last_tick_.store(tick);
        have_tick_.store(true);
        if (had_tick && delta < 0) {
            clock_invalid_.store(true);
            receipt_ns_.store(0);
        }
        if (!advances) return;
        receipt_ns_.store(steady_now_ns());
    }

private:
    std::atomic<uint64_t> receipt_ns_{0};
    std::atomic<uint32_t> last_tick_{0};
    std::atomic<bool> have_tick_{false};
    std::atomic<bool> clock_invalid_{false};
};
}  // namespace

std::unique_ptr<LowCmd_t> FSMState::lowcmd = nullptr;
std::shared_ptr<LowState_t> FSMState::lowstate = nullptr;
std::shared_ptr<Keyboard> FSMState::keyboard = nullptr;

void init_fsm_state(bool simulator)
{
    auto startup_lowstate = std::make_shared<StartupLowState>();
    FSMState::lowstate = startup_lowstate;
    go2_shutdown_receipt_ns = [startup_lowstate] { return startup_lowstate->receipt_ns(); };
    if (simulator) {
        spdlog::warn("Simulator startup explicitly selected: MotionSwitcher/StandDown handoff is bypassed");
        bool connected = false;
        for (int poll = 0; poll < 750 && keep_running; ++poll) {
            const uint64_t receipt = startup_lowstate->receipt_ns();
            const uint64_t now = steady_now_ns();
            if (receipt != 0 && receipt <= now && now - receipt <= 100'000'000) {
                connected = true;
                break;
            }
            usleep(20000);
        }
        if (!connected)
            throw std::runtime_error("simulator LowState did not become fresh before timeout; no LowCmd publisher was created");
        FSMState::lowcmd = std::make_unique<LowCmd_t>();
        return;
    }
    auto lowcmd_sub = std::make_shared<unitree::robot::go2::subscription::LowCmd>();

    unitree::robot::b2::MotionSwitcherClient motion_switcher;
    motion_switcher.SetTimeout(3.0f);
    motion_switcher.Init();
    unitree::robot::go2::SportClient sport;
    sport.SetTimeout(5.0f);
    sport.Init();

    const auto poses = param::config["FSM"]["FixStand"]["qs"].as<std::vector<std::vector<float>>>();
    if (poses.size() < 2 || poses[1].size() != 12)
        throw std::runtime_error("FixStand.qs[1] must contain the 12-joint down pose");

    Go2StartupHandoffConfig config;
    std::copy(poses[1].begin(), poses[1].end(), config.down_q.begin());
    // StandDown may leave the ab/adduction joints spread while the thigh and calf
    // joints still distinguish the down pose clearly from standing.
    config.joint_tolerance.fill(0.25f);
    for (size_t leg = 0; leg < 4; ++leg) config.joint_tolerance[leg * 3] = 0.50f;

    Go2StartupHandoffHooks hooks;
    hooks.check_mode = [&motion_switcher](std::string& form, std::string& name) {
        return motion_switcher.CheckMode(form, name);
    };
    hooks.stand_down = [&sport] { return sport.StandDown(); };
    hooks.release_mode = [&motion_switcher] { return motion_switcher.ReleaseMode(); };
    hooks.sample_lowstate = [startup_lowstate] {
        Go2StartupSample sample;
        std::lock_guard<std::mutex> lock(startup_lowstate->mutex_);
        sample.receipt_ns = startup_lowstate->receipt_ns();
        sample.tick = startup_lowstate->msg_.tick();
        for (size_t i = 0; i < sample.q.size(); ++i) {
            sample.q[i] = startup_lowstate->msg_.motor_state()[i].q();
            sample.dq[i] = startup_lowstate->msg_.motor_state()[i].dq();
        }
        const auto& quaternion = startup_lowstate->msg_.imu_state().quaternion();
        std::copy_n(quaternion.begin(), sample.quaternion.size(), sample.quaternion.begin());
        return sample;
    };
    hooks.competing_lowcmd_active = [lowcmd_sub] { return !lowcmd_sub->isTimeout(); };
    hooks.cancelled = [] { return !keep_running; };
    hooks.now_ns = steady_now_ns;
    hooks.wait_poll = [] { usleep(20000); };

    spdlog::info("Startup handoff: checking sport mode and requiring a stable down pose before LowCmd startup");
    const auto result = go2_perform_startup_handoff(config, hooks);
    if (!result.ready) {
        throw std::runtime_error(result.error +
                                 "; no LowCmd publisher was created. Use --sim --network lo only for the explicit simulator path.");
    }

    FSMState::lowcmd = std::make_unique<LowCmd_t>();
    spdlog::info("Startup handoff complete: sport mode is inactive, the robot is down, and rt/lowcmd is quiet");
}

int main(int argc, char** argv)
{
    // Ctrl+C also terminates a foreground `tee`. If its pipe closes, logging
    // must not terminate the controller with SIGPIPE before StandDown finishes.
    // Install before any output (including keyboard help and startup logging).
    std::signal(SIGPIPE, SIG_IGN);
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
        std::signal(SIGHUP, stop_on_signal);
        while (keep_running) {
            terminal.poll();
            usleep(10000);
        }
        return 0;
    }

    // Load parameters
    auto vm = param::helper(static_cast<int>(forwarded.size()), forwarded.data());

    if (options.simulator && vm["network"].as<std::string>() != "lo") {
        std::cerr << "--sim is restricted to --network lo; DDS was not initialized.\n";
        return 2;
    }

    std::cout << " --- Unitree Robotics --- \n";
    std::cout << "     Go2 Controller \n";

    std::unique_ptr<Go2TerminalInput> terminal;
    if (options.keyboard) {
        go2_keyboard_control = std::make_shared<Go2KeyboardControl>();
        terminal = std::make_unique<Go2TerminalInput>(*go2_keyboard_control);
        Go2TerminalInput::print_help();
    }
    std::signal(SIGINT, stop_on_signal);
    std::signal(SIGTERM, stop_on_signal);
    std::signal(SIGHUP, stop_on_signal);

    // Unitree DDS Config
    unitree::robot::ChannelFactory::Instance()->Init(0, vm["network"].as<std::string>());

    try {
        init_fsm_state(options.simulator);
    } catch (const std::exception& error) {
        spdlog::critical("Controller startup aborted: {}", error.what());
        terminal.reset();
        return 1;
    }

    const auto down_pose = param::config["FSM"]["FixStand"]["qs"][1].as<std::vector<float>>();
    if (down_pose.size() != go2_shutdown_down_q.size())
        throw std::runtime_error("shutdown down pose must contain 12 joints");
    std::copy(down_pose.begin(), down_pose.end(), go2_shutdown_down_q.begin());

    go2_gyro_bias = std::make_unique<Go2GyroBiasSubscriber>();

    // Initialize FSM
    auto fsm = std::make_unique<CtrlFSM>(param::config["FSM"]);
    fsm->start();

    if (!options.keyboard) {
        std::cout << "Press [L2 + A] to enter FixStand mode.\n";
        std::cout << "And then press [Start] to start controlling the robot.\n";
    }

    while (keep_running) {
        if (terminal) terminal->poll();
        usleep(10000);
    }
    go2_shutdown.request();
    spdlog::warn("Shutdown requested: Policy -> settled Stand -> StandDown; "
                 "waiting for 0.5 s measured down settling and another 1 s hold. Control output remains active.");
    auto next_notice = std::chrono::steady_clock::now() + std::chrono::seconds(15);
    while (!go2_shutdown.complete()) {
        if (std::chrono::steady_clock::now() >= next_notice) {
            spdlog::warn("Shutdown still waiting: down pose not confirmed. Keeping control active; "
                         "check posture, joint settling and LowState connection. Repeated signals do not force exit.");
            next_notice += std::chrono::seconds(5);
        }
        usleep(10000);
    }
    spdlog::info("Shutdown: measured down pose settled for 0.5 s and held another 1 s; exiting controller");
    // CtrlFSM has no stop API. Restore the terminal, then terminate the process without
    // racing its recurrent control thread against state destruction.
    terminal.reset();
    std::_Exit(128 + shutdown_signal);
}
