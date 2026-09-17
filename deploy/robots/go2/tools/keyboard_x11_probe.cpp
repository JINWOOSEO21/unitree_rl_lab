// Read-only X11 steering diagnostic: no DDS, no LowCmd, no FSM, no motor output.
//
// --keyboard-check cannot exercise a/d hold steering, because Go2TerminalInput clears
// the held-heading state whenever the control is not in Policy and set_heading_keys()
// early-returns outside Policy. This probe drives the same Go2HeldHeadingInput and
// Go2KeyboardControl with the state forced to Policy, so a/d press/release, both-keys,
// and focus loss can be checked on a real X11 session before any robot is powered.
//
// Run it from a focused terminal in the session under test:
//   ./go2_keyboard_x11_probe
// Press a, d, both, release each, then click away and back. Ctrl+C exits.

#include <X11/Xlib.h>

#include <atomic>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <unistd.h>

#include "HeldHeadingInput.h"
#include "KeyboardControl.h"

namespace
{
volatile std::sig_atomic_t keep_running = 1;
void stop_on_signal(int) { keep_running = 0; }

const char* env_or(const char* name, const char* fallback)
{
    const char* value = std::getenv(name);
    return (value && *value) ? value : fallback;
}

constexpr float kRadToDeg = 57.2957795f;
}  // namespace

int main()
{
    std::printf("[x11-probe] NO DDS / NO MOTOR OUTPUT. Read-only key state inspection.\n");
    std::printf("[x11-probe] XDG_SESSION_TYPE=%s DISPLAY=%s WAYLAND_DISPLAY=%s\n",
                env_or("XDG_SESSION_TYPE", "<unset>"), env_or("DISPLAY", "<unset>"),
                env_or("WAYLAND_DISPLAY", "<unset>"));

    // Report the focus window the same way Go2HeldHeadingInput latches it at construction.
    if (Display* display = XOpenDisplay(nullptr)) {
        Window focus = None;
        int revert = 0;
        XGetInputFocus(display, &focus, &revert);
        const char* kind = focus == None          ? " (None: no X client holds focus)"
                           : focus == PointerRoot ? " (PointerRoot: focus follows pointer)"
                                                  : "";
        std::printf("[x11-probe] XGetInputFocus window=0x%lx%s\n", focus, kind);
        XCloseDisplay(display);
    } else {
        std::printf("[x11-probe] XOpenDisplay failed: no X server reachable.\n");
    }

    Go2HeldHeadingInput held;
    if (!held.available()) {
        std::printf(
            "[x11-probe] RESULT: X11 steering UNAVAILABLE. a/d hold steering would be disabled.\n"
            "[x11-probe] Log in to a real Xorg session (GDM gear icon -> \"Ubuntu on Xorg\")\n"
            "[x11-probe] and rerun this probe from a focused terminal in that session.\n");
        return 1;
    }
    std::printf("[x11-probe] X11 steering available. Focus this terminal, then:\n");
    std::printf("[x11-probe]   hold a -> expect left/+15 deg, release -> 0\n");
    std::printf("[x11-probe]   hold d -> expect right/-15 deg, release -> 0\n");
    std::printf("[x11-probe]   hold a+d together -> expect 0 deg (they cancel)\n");
    std::printf("[x11-probe]   KEEP HOLDING a and click another window ->\n");
    std::printf("[x11-probe]     expect FOCUS LOST then 0 deg while the key is still down\n");
    std::printf("[x11-probe] Ctrl+C to exit.\n\n");

    Go2KeyboardControl control;
    control.set_state(Go2RuntimeState::Policy);  // set_heading_keys() is a no-op outside Policy.

    std::signal(SIGINT, stop_on_signal);
    std::signal(SIGTERM, stop_on_signal);
    std::signal(SIGHUP, stop_on_signal);

    bool armed = false;
    Go2HeldHeadingKeys previous{};
    bool previous_valid = false;
    float previous_offset = 0.0f;
    bool previous_focus = true;

    while (keep_running) {
        // Report focus separately: poll() drops held keys on focus loss, which would
        // otherwise be indistinguishable from the user releasing the key.
        const bool focus = held.focused();
        if (focus != previous_focus) {
            std::printf("[x11-probe] === FOCUS %s ===\n", focus ? "REGAINED" : "LOST");
            std::fflush(stdout);
            previous_focus = focus;
        }

        // Go2TerminalInput arms on the a/d keypress byte; re-arm whenever the probe is idle
        // so a hold that starts while unarmed is still observed.
        if (!armed) {
            held.arm();
            armed = true;
        }
        const auto keys = held.poll();
        control.set_heading_keys(keys.left, keys.right);
        const float offset = control.axes().heading_offset;

        const bool changed = !previous_valid || keys.left != previous.left ||
                             keys.right != previous.right || offset != previous_offset;
        if (changed) {
            std::printf("[x11-probe] a=%d d=%d -> heading_offset=%+.4f rad (%+.1f deg)%s\n",
                        static_cast<int>(keys.left), static_cast<int>(keys.right), offset,
                        offset * kRadToDeg, focus ? "" : "   [unfocused]");
            std::fflush(stdout);
            previous = keys;
            previous_offset = offset;
            previous_valid = true;
        }
        if (!keys.left && !keys.right) armed = false;  // poll() clears the arm on full release.
        usleep(10000);                                 // 100 Hz, same order as the control loop.
    }

    std::printf("\n[x11-probe] exit. No DDS was initialized and no LowCmd was sent.\n");
    return 0;
}
