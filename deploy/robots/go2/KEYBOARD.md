# Go2 keyboard controls

Run `./go2_ctrl --network enp42s0 --keyboard` in a focused local X11 terminal.
The build uses the installed X11 development library. SSH/headless terminals
without an accessible X11 display explicitly disable a/d; other keys still work.

- `1`: transition to standing (about 2 seconds).
- `2`: enter policy after readiness checks.
- Hold `a`: command a heading error of +15 degrees relative to current body yaw.
- Hold `d`: command -15 degrees relative to current body yaw.
- Release both (or hold both): command zero heading error, continuing forward.
- `a/d` cancel the previous continuous `q/e` turn. They do not change speed.
- `q/e`: original persistent turn-rate input, -/+0.25 per terminal key event.
- Space: clear commands and return from Policy to Stand.
- `3`: controlled descent from settled Stand (about 3 seconds).
- `0`: Passive damping, not a controlled descent.

Holding a/d does not accumulate a fixed world heading: the reference follows
current body yaw. Releasing follows the forward direction at release, not the
orientation before the turn. Existing observation heading scale (1.5) remains,
so a geometric 15-degree error is supplied as approximately 0.3927 in the policy
observation. Robot motion depends on the policy, rather than exactly following
that angle instantaneously. A held offset is a steering command, not strafing.

Key state is queried every terminal poll (nominally 10 ms), then applied at the
next policy step. This is not a hard real-time release bound. Activation needs
an a/d byte in this terminal plus the actual X11 key state. Window focus loss
clears the hold; focus return alone does not reactivate it. Mode changes clear
pending holds. X11 access is read-only, with no key grab or event injection.

Reference: [Xlib XQueryKeymap](https://xorg.freedesktop.org/archive/current/doc/libX11/libX11/libX11.html#XQueryKeymap).
