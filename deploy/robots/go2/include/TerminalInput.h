#pragma once

#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <stdexcept>
#include <string>
#include <termios.h>
#include <unistd.h>

#include "KeyboardControl.h"

class Go2TerminalInput
{
public:
    explicit Go2TerminalInput(Go2KeyboardControl& control) : control_(control)
    {
        if (!isatty(STDIN_FILENO))
            throw std::runtime_error("--keyboard requires an interactive terminal on stdin");
        install_job_control_ignores();
        foreground_ = is_foreground();
        if (!foreground_)
            throw std::runtime_error("--keyboard requires this process to own the terminal foreground");
        if (foreground_) enable_raw();
        if (!raw_active_) throw std::runtime_error("failed to place stdin in nonblocking raw mode");
    }

    ~Go2TerminalInput()
    {
        restore();
        restore_job_control_handlers();
    }

    Go2TerminalInput(const Go2TerminalInput&) = delete;
    Go2TerminalInput& operator=(const Go2TerminalInput&) = delete;

    void poll()
    {
        const bool now_foreground = is_foreground();
        if (!now_foreground) {
            if (foreground_) control_.foreground_lost();
            foreground_ = false;
            restore();
            return;
        }
        if (!foreground_) {
            foreground_ = true;
            enable_raw();
        }
        if (!raw_active_) enable_raw();
        if (!raw_active_) {
            control_.foreground_lost();
            return;
        }

        char key = 0;
        const ssize_t count = ::read(STDIN_FILENO, &key, 1);
        if (count == 1) {
            if (discard_escape_byte(static_cast<unsigned char>(key))) return;
            control_.handle_key(key);
            print_event(key);
        } else if (count < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
            control_.foreground_lost();
            restore();
        }
    }

    static void print_help()
    {
        std::puts(
            "\n[Go2 keyboard]\n"
            "  1: measured pose -> stand (2 s)\n"
            "  2: enter policy only after stand completes and fresh valid scandots arrive\n"
            "  0: Passive (damping control)\n"
            "  w/s: sticky forward-speed command +/-0.25 (policy minimum remains configured)\n"
            "  q/e: sticky left/right turn command -/+0.25\n"
            "  space: reset speed/turn; while Policy, return to Stand over 2 s\n"
            "  h: show this help\n"
            "  Lateral a/d and a backward command are not implemented for this policy.\n"
            "  Keep this process in the terminal foreground; losing it returns Policy to Stand.");
    }

private:
    bool is_foreground() const
    {
        const pid_t group = tcgetpgrp(STDIN_FILENO);
        return group >= 0 && group == getpgrp();
    }

    void enable_raw()
    {
        if (raw_active_) return;
        if (tcgetattr(STDIN_FILENO, &saved_) != 0) return;
        termios raw = saved_;
        raw.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
        raw.c_cc[VMIN] = 0;
        raw.c_cc[VTIME] = 0;
        if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) == 0) raw_active_ = true;
    }

    void restore()
    {
        if (!raw_active_) return;
        tcsetattr(STDIN_FILENO, TCSANOW, &saved_);
        raw_active_ = false;
    }

    void install_job_control_ignores()
    {
        struct sigaction ignore{};
        ignore.sa_handler = SIG_IGN;
        sigemptyset(&ignore.sa_mask);
        sigaction(SIGTTOU, &ignore, &old_sigttou_);
        sigaction(SIGTTIN, &ignore, &old_sigttin_);
        handlers_installed_ = true;
    }

    void restore_job_control_handlers()
    {
        if (!handlers_installed_) return;
        sigaction(SIGTTOU, &old_sigttou_, nullptr);
        sigaction(SIGTTIN, &old_sigttin_, nullptr);
        handlers_installed_ = false;
    }

    void print_event(char key)
    {
        if (key == 'h') {
            print_help();
            return;
        }
        const auto axes = control_.axes();
        if (key == 'w' || key == 's' || key == 'q' || key == 'e' || key == ' ')
            std::printf("[key] %s speed=%+.2f turn=%+.2f\n", key == ' ' ? "space" : std::string(1, key).c_str(), axes.speed, axes.turn);
        else if (key >= 0x20 && key < 0x7f)
            std::printf("[key] %c\n", key);
        std::fflush(stdout);
    }

    bool discard_escape_byte(unsigned char key)
    {
        const auto now = std::chrono::steady_clock::now();
        if (escape_state_ != EscapeState::None && now > escape_deadline_)
            escape_state_ = EscapeState::None;
        if (key == 0x1b) {
            escape_state_ = EscapeState::Escape;
            escape_deadline_ = now + std::chrono::milliseconds(50);
            return true;
        }
        if (escape_state_ == EscapeState::None) return false;
        escape_deadline_ = now + std::chrono::milliseconds(50);
        if (escape_state_ == EscapeState::Escape) {
            if (key == '[') escape_state_ = EscapeState::Csi;
            else if (key == 'O') escape_state_ = EscapeState::Ss3;
            else escape_state_ = EscapeState::None;
            return true;
        }
        if (escape_state_ == EscapeState::Ss3 || (key >= 0x40 && key <= 0x7e))
            escape_state_ = EscapeState::None;
        return true;
    }

    enum class EscapeState { None, Escape, Csi, Ss3 };

    Go2KeyboardControl& control_;
    termios saved_{};
    bool raw_active_ = false;
    bool foreground_ = false;
    bool handlers_installed_ = false;
    struct sigaction old_sigttou_{};
    struct sigaction old_sigttin_{};
    EscapeState escape_state_ = EscapeState::None;
    std::chrono::steady_clock::time_point escape_deadline_{};
};
