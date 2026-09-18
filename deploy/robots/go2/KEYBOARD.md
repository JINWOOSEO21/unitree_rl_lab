# Go2 keyboard controls

Run `./go2_ctrl --network enp42s0 --keyboard` in a focused local X11 terminal.
The build uses the installed X11 development library. SSH/headless terminals
without an accessible X11 display explicitly disable a/d; other keys still work.

- `1`: transition to standing (about 2 seconds).
- `2`: enter policy after readiness checks.
- `i/k`: increase/decrease the persistent forward-speed input by 0.25, formerly
  w/s. The policy's configured minimum forward speed still applies.
- `w`: reset the body-relative target angle to zero and print it. Speed and
  mode are unchanged; this continues straight rather than stopping the robot.
- `s`: unbound.
- Hold `a`: command a heading error of +15 degrees relative to current body yaw.
- Hold `d`: command -15 degrees relative to current body yaw.
- Release both (or hold both): command zero heading error, continuing forward.
- `a/d` cancel the previous `q/e` heading offset. They do not change speed.
- `q/e`: increase/decrease the current body-relative target angle by 10 degrees
  per terminal key event; print only that target in degrees (left positive).
  Example: q, q, e prints +10, +20, +10. The angle persists after release,
  wraps within +/-180 degrees, and is not relative to Policy-entry orientation.
  Holding a key may generate repeated 10-degree changes via terminal auto-repeat.
  A nonzero body-relative target continues steering as the body rotates; it does
  not automatically become zero after the robot turns that many degrees.
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

## Controlled shutdown

Ctrl+C (SIGINT), SIGTERM, and SIGHUP request shutdown; the signal handler only
sets a flag. The FSM and LowCmd publisher keep running during this sequence:

- Policy -> FixStand, using the existing posture transition.
- Stand -> StandDown only after the normal upright / joint-settling gate passes.
- StandDown completes its configured 3 s trajectory, then fresh, advancing
  LowState must confirm the folded joint pose with low joint speed for 0.5 s, followed by another 1 s hold.
- Already Passive: no stand-up is commanded; exit only when the measured folded
  pose is confirmed for 0.5 s and held another 1 s. Existing StandDown finishes without restarting.

Down confirmation uses the startup folded-pose envelope (hip error <=0.50 rad,
thigh/calf <=0.25 rad), joint speed <=0.2 rad/s and body tilt <=0.3 rad. It is a
joint/IMU check, not a physical ground-contact certificate. Duplicate or regressing
ticks, gaps over 100 ms, bad measurements, or posture deviations reset the hold.

Typical completion from settled Stand is about 4.5 s; from Policy about 6.5–7.5 s,
plus actual settling time. No fixed timer cuts motor output while standing.
If confirmation fails, the process remains alive and reports a waiting message
(after 15 s, then every 5 s). Existing sensor/orientation/terrain fault routes
still take priority and can lead to Passive; shutdown does not automatically
re-energize a Passive robot. Repeated exit signals do not bypass confirmation.
Keyboard/joystick mode commands are suppressed while shutdown is pending.

Logging through `... 2>&1 | tee walk.log` also supports controlled shutdown.
Ctrl+C reaches both processes, so `tee` can exit before the descent finishes.
The controller ignores SIGPIPE: a closed stdout/stderr pipe must not interrupt
motor output or the down-pose confirmation. Subsequent messages may be absent
from the terminal and tee log; add `--log` to retain them in `log/log.txt`.
The PTY regression test covers closed log pipes and SIGINT/SIGTERM/SIGHUP in
the real binary's no-DDS mode; state-machine tests cover the descent gates.

SIGKILL (kill -9), crashes, machine/power loss, and external supervisors that
force-kill after a timeout cannot run this shutdown sequence. Normal terminal
closure commonly sends SIGHUP, but that depends on the terminal/supervisor.
These changes have automated state-machine tests; actual robot descent on a
shutdown signal still requires a supervised hardware check.

## Shared gyro bias

The leg bridge publishes `rt/parkour/gyro_bias` (`std_msgs::msg::dds_::String_`,
JSON) at a nominal 5 Hz when run with `--odom leg --publish-scandots`. No new
command-line argument is needed. Payload version 1 contains a bridge session
UUID, increasing sequence, calibrated flag, three `bias_rad_s` values,
`source_tick`, `frame_id=base_link`, and `units=rad/s`.

The bridge sends uncalibrated status on startup, on stale LowState (>100 ms),
and on normal shutdown. It reuses the existing stationary calibration; it does
not estimate a second bias. Controller startup alone requires no recalibration:
it can receive the next calibrated heartbeat from the running bridge.

Policy entry requires a calibrated heartbeat received within 1 second, finite
bias components within +/-0.1 rad/s, and advancing source ticks. A new session's
uncalibrated message immediately invalidates the previous candidate. Duplicate,
old-sequence, or retired-session messages cannot refresh validity. Keyboard `2`
and joystick entry both use this gate, alongside existing stand/map checks.

At entry, the controller copies the bias and logs its session, tick and values.
The policy uses `(latest LowState gyro - frozen bias) * 0.25` in prop[0:3] and
history. The snapshot does not change during that Policy run, even if the bridge
restarts or sends a new calibration. Stand/re-entry uses the latest valid sample.
A race where validity disappears at actual entry cancels inference and returns
to Stand. There is no silent zero-bias fallback. Quaternion is not modified.

A bridge restart still interrupts scandots; existing terrain fault behavior
remains. Restart the bridge outside Policy. Robot/LIO-only bridges do not provide
this calibration stream yet, so cannot alone satisfy the new Policy entry gate.
The same gate also applies in simulation; provide a calibration publisher there.

Verification includes cache/session/timeout/invalid payload tests, entry gating,
corrected observation/history tests, and a Python-to-C++ DDS probe restricted to
loopback interface `lo`, domain 181. No hardware walking verification is implied.
