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
