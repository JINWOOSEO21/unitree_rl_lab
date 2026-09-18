# Go2 MIT-style proprioceptive odometry

`go2_sensor_bridge.py --odom mit` implements the two-stage estimator in
Bledt et al., *MIT Cheetah 3: Design and Control of a Robust, Dynamic Quadruped
Robot*, section III-H (PDF page 6). Reference implementation:
https://github.com/mit-biomimetics/Cheetah-Software/blob/master/common/src/Controllers/PositionVelocityEstimator.cpp

## Run on the Jetson

```bash
source ~/walking/env.sh
python "$GO2_PARKOUR/tools/go2_sensor_bridge.py" \
  --interface eth0 --odom mit --emcupy-root "$GO2_EMCUPY" --summary-only
```

Stand still with all four feet loaded for the initial 10-second body gyro
calibration. The default invocation prints diagnostics and maps; it does not
publish terrain to the controller. Existing `--publish-scandots` opts into the
same terrain output and body gyro-bias heartbeat used by `--odom leg`.
`--duration 30` bounds a diagnostic run. `--check-dependencies` checks imports
without joining DDS. The default odometry remains `leg`.

The estimator uses NumPy and the existing Go2 geometry contract. There is no new
Python package, ROS node, Point-LIO process, or L1 IMU input to start. Mapping
continues to require the existing PyTorch/CuPy environment and L1 cloud.

## Sensors and algorithm

Only `rt/lowstate` supplies odometry inputs:

- Go2 **body IMU** quaternion (initialization and gap recovery), gyroscope in
  rad/s, accelerometer specific force in m/s².
- Twelve measured joint angles and velocities, converted from SDK to contract
  order. Analytic Jacobian products supply foot linear/angular velocity.
- Four raw foot-force readings for contact detection. These are thresholded
  raw hardware values, not assumed calibrated Newton measurements.

The **L1 built-in IMU is not used**. The body IMU axes/origin follow the existing
body/base convention. Body gyro bias uses the existing stationary calibration;
no acceleration bias or scale is estimated or subtracted.

Attitude is integrated using bias-corrected gyro, with a weak gravity-direction
correction (gain 0.1/s, reduced when acceleration magnitude differs from g).
The LowState quaternion initializes this filter; it is not copied at each step.
Yaw has no external correction. After a source-time gap, measured attitude is
reacquired and the terrain map is cleared.

A separate 18-state linear Kalman filter estimates base position/velocity and
four world foot-link origins. Accelerometer specific force is rotated into a
gravity-aligned frame and gravity is added. Stance-foot relative position and
zero-contact-point velocity provide observations; airborne feet do not provide
these observations. Touchdown initializes a new foot state with base/kinematic
uncertainty. A Joseph covariance update preserves numerical symmetry/PSD.

Go2 adaptations (not a literal copy of MIT's controller code):

- No `foot_height = 0` measurement: independent foot heights permit stairs and
  uneven ground without an externally known contact plane.
- Contact-force filtering, 20 ms settling, and a 20 ms confidence ramp replace
  MIT's scheduled gait-phase input. Measured contact is used, not motor commands.
- Foot rolling uses the existing contract's 0.0234 m radius and world vertical.
  That radius comes from the existing simulation geometry and is not newly
  calibrated on hardware. Slopes and foot slip remain error sources.
- Prediction without support is accepted for at most 0.15 s after last support.
  Thereafter poses are invalid for mapping and the map is cleared. Repeated
  LowState gaps exhaust the same budget as the leg adapter.
- `--mit-odom-hz` controls MIT (default 75 Hz); `--leg-odom-hz` still controls
  leg (default 100 Hz). The paper's 1 kHz controller period is not assumed.
  Source tick determines dt. With the fixed 20 ms preceding-pose freshness
  limit, 50 Hz leaves no scheduling margin and caused more stale-pose rejects
  in the measured runs. That freshness limit has not been relaxed.

## Coordinates and output

`mit_odometry` in LowState diagnostic rows and `odometry` in scan rows give the
current **base origin** relative to the base frame at filter initialization,
after stationary calibration. They carry `reference_frame: initial_robot_base`.
Position and orientation are both rebased; there is no L1 lever arm to remove.

Elevation mapping requires gravity-aligned Z, including when the initial robot
is tilted. `mapping_pose` explicitly carries the equivalent gravity-aligned
pose with its origin at the same initial base position. Its `reference_frame`
is `gravity_aligned_odom`. The bridge passes this pose to the mapper and uses
its position in the existing scandots protocol. The fixed `frame_id: odom`
strings are compatibility labels for the bridge's local JSON pose contract;
these nested records are not two TF transforms with the same frame name.
`initial_base_orientation_in_mapping_wxyz` records the constant rotation
between the two coordinate conventions. Do not feed the initial-base pose
directly to a height mapper that assumes gravity-aligned Z.

MIT uses polling CycloneDDS readers with KeepLast(1). Each owned reader thread
is stopped before its DDS objects are released, avoiding the observed SDK
listener deletion deadlock. Existing leg and robot subscribers are unchanged.
No system/SDK file or network configuration is modified.

After warmup and before subscribers start, MIT collects then freezes the
initialized Python heap for the duration of the bridge run. This avoids a
measured ~155 ms full-GC scan over long-lived imported objects. New cyclic
objects are still collected normally; GC is not disabled. Normal GC tracking
is restored on exit, and a caller's existing frozen generation is respected.

## Validation

Run in the Jetson walking environment:

```bash
PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 python -m unittest discover \
  -s "$GO2_PARKOUR/tools/tests" -p 'test_go2_mit_pose.py' -v
```

Tests cover FK/Jacobian consistency, known translation with unequal foot heights,
short ballistic flight and support-loss invalidation, gyro bias, yaw integration,
swing/landing transitions, covariance PSD, missing/nonfinite acceleration,
source-clock gaps/regression, initial-base conversion, and bridge routing to the
gravity-aligned mapper. Existing leg and sensor-bridge tests are also run.

The bounded live checks only subscribe and generate local terrain diagnostics;
no motor commands, sport RPCs, or production terrain publishing are performed.
Stationary validation is not a validation of walking, turning, sliding, jumping,
or stair accuracy. Those require moving data before comparing against `leg`.
The estimator has no external position/yaw correction and does not guarantee
lower drift than `leg`.

### Jetson results (2026-09-18)

Ubuntu 20.04 / L4T R35.3.1, existing walking Python environment; no new packages.

- 12 MIT tests, 19 existing leg adapter tests and 16 sensor bridge tests passed.
- Previously recorded 25 s stationary capture: 12,609 LowState samples,
  2,426 accepted updates, 1,456 valid MIT poses after calibration, no source
  gaps. Maximum position norm 3.30 mm (existing leg on the same capture: 2.06 mm).
  MIT update p50/p95/max: 3.09/3.18/4.82 ms with default BLAS environment after
  replacing the tiny matrix solve with equivalent scalar observations.
- Before the subsequent GC/rate tuning, a 30 s live run: normal exit, 12,709 LowState samples, 2,880 estimator
  updates, 1,341 valid poses, 136 local terrain scans, maximum stationary
  position norm 1.69 mm. Valid scan age p50/p95/max: 113/163/181 ms.
- One 142 ms LowState gap occurred during startup; the calibration window
  restarted and the bridge recovered. One stale pose was rejected. The other
  159 rejected map attempts were before a valid calibrated/support pose was
  available. There were no fatal errors. Startup therefore took longer than
  the nominal 10 seconds: the calibration requires an uninterrupted window.

Raw logs and summaries remain on the Jetson under `~/walking/mit_runtime/`
(`live_04.jsonl`, `live_04_summary.json`, `replay_scalar.json`). Large sensor logs
are not part of the source commit. Earlier runs exposed listener shutdown hangs
and occasional mapping deadlines; the MIT polling path and scalar update are
included in the implementation. No claim of improved walking accuracy is
made from these stationary runs.

### Scheduling diagnosis and final default

A separate profile records accepted estimator cost, map computation, scan age,
LowState arrival gaps and `gc.callbacks`. These are wall times on the Jetson
with the map active, unlike the faster offline replay timing above. Input
latency relative to the fastest observed arrival is only a jitter measurement;
robot and host clocks are not synchronized for absolute sensor latency.

Before GC tuning, a 30 s MIT 100 Hz run had a 156 ms LowState gap coincident
with a 155 ms generation-2 GC scan. MIT 50 Hz had the same GC problem and 18
stale-pose rejects, versus one at 100 Hz. Existing leg 100 Hz also had a
143 ms GC scan and delayed LowState delivery. Reducing estimator frequency
alone therefore did not address the long pause.

With initialized-heap GC handling, MIT at the new 75 Hz default ran for 60 s:

- Calibration completed at 10.03 s with no restart or source-gap fault.
- 499 terrain scans over the remaining approximately 50 s; one stale-pose
  reject, no stale-cloud or stale-LowState-after-mapping faults, no fatal errors.
- Estimator p50/p95/max: 3.90/5.02/8.45 ms.
- Map computation p50/p95/max: 45.7/90.5/110.2 ms.
- Valid scan age p50/p95/max: 86.7/133.9/162.6 ms, below the unchanged 200 ms limit.
- No GC pause above 5 ms; maximum LowState interarrival gap 33.4 ms.

The 75 Hz setting reduces estimator work by roughly one quarter relative to
100 Hz while keeping nominal pose intervals below the 20 ms freshness limit.
Actual updates are quantized to available source ticks (often about 14 ms).
This is scheduling headroom demonstrated in stationary tests, not a hard
real-time guarantee or a walking-accuracy validation. Profile logs are in
`~/walking/mit_runtime/profile_*.json`; the `mit100_gc` run overlapped startup of
another profiler and is excluded from comparisons.

A subsequent isolated 30 s run at 100 Hz with the same GC handling completed
calibration at 10.03 s, but produced 188 scans, one stale-pose reject and one
stale-cloud-after-mapping reject. Map computation p95/max was 127/163 ms;
valid scan age p95/max was 172/198 ms. No GC pause exceeded 5 ms. Compared with
the 75 Hz run, this supports keeping 75 Hz as the default to leave more
scheduling margin; it does not imply that every 100 Hz update misses a deadline.
