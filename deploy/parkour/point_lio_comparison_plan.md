# Go2: leg odometry / Point-LIO comparison implementation plan

Date: 2026-09-16. Status: planning and subscriber-only measurements complete;
no Point-LIO installation, robot motion, sport-mode change, or product code change.

## Objective and boundaries

Run the existing bias-corrected leg estimator and official Unitree Point-LIO on
the same input recording, evaluate both, then permit an explicit mapping pose
source selection. Keep SDK2 C++ go2_ctrl and Python elevation mapping roles.
No automatic odometry fallback, switching, or blending in the initial version.
Do not send experimental terrain to the policy's rt/parkour/scandots topic.

## Evidence collected now

Evidence directory: `captures/lio_feasibility_20260916/`: `probe.py`,
`samples.jsonl`, `cloud.bin`, `summary.json`, `timing_summary.json`.
Twenty-second subscriber-only measurement (capture is stopped):

| Input | Result |
|---|---|
| MotionSwitcher.CheckMode | code 0, form 0, name mcf; sport active during this measurement |
| rt/utlidar/cloud | 309 messages, 15.402 Hz, frame utlidar_lidar |
| Cloud fields | x/y/z/intensity float32, ring uint16, time float32; point_step 32 |
| Cloud point count | mean 4168.76 |
| Relative point time span | median 62.533 ms, max 89.817 ms, assuming seconds |
| rt/utlidar/imu | 4979 messages, 248.653 Hz, sensor_msgs::msg::dds_::Imu_, frame utlidar_imu |
| IMU header gaps | median 4.472 ms; max 34.708 ms; no duplicate/regression |
| LiDAR IMU acceleration mean | [3.603018, -0.000867, 9.805880]; norm 10.446866 |
| LiDAR IMU gyro mean | [-0.005265, 0.000556, 0.000609] |
| rt/lowstate | about 498.7 Hz; q changes very small, consistent with stationary standing |
| rt/utlidar/robot_odom | about 148.46 Hz; reference only, not ground truth |
| Alternate rt/utlidar/imu_data | no samples observed; this does not prove permanent absence |
| Desktop | Ubuntu 24.04.4, no /opt/ros, Docker server 29.8.0 available |

No callback decode errors; a reader shutdown message occurred when closing DDS.
Rates describe this bounded probe, not a guaranteed hardware specification.
A common header epoch and full-range IMU coverage of 308/309 cloud intervals
support feasibility but DO NOT prove hardware synchronization. One cloud's
point-time endpoint exceeds the next header by >1 ms. Inspect original sensor
clock/driver semantics before deciding whether this is overlap, jitter, or loss.

## Decision 1: preserve official algorithm as baseline

Use an isolated Ubuntu 20.04/ROS1 Noetic container on the desktop. Pin image
DIGEST, dependencies and Point-LIO commit `18ed5976d8fab2bd8a5148c26a40692bd3c0dc91`
(v2.0.2 baseline); record compiler/build/config metadata. Do not install ROS into
conda or change current numpy/CuPy/SDK dependencies. Noetic is EOL, so keep the
baseline isolated and move to a maintained environment only after equivalence
is demonstrated. Do not start a competing serial LiDAR driver on Jetson.

Alternative: ROS2 port on Ubuntu 24.04. Good maintenance destination, but adds
porting errors before input correctness is known. Defer, not reject permanently.
Run CPU Point-LIO initially; measure CPU/latency alongside GPU elevation mapping.
Docker/network/interface compatibility must be tested, not presumed from docker
version alone. Use explicit enp42s0 DDS domain 0 for sensor subscriptions; do not
create LowCmd publishers or motion clients in this evaluation container.

## Stage A: establish the actual input contract (first implementation work)

1. Promote the temporary probe into a bounded capture/replay utility. Prefer the
   supplied SDK2 C++ Imu_ IDL for production; local SDK2 Python lacks Imu_. The
   temporary measurement class mirrored the supplied C++ IDL only for this probe.
2. Capture raw cloud bytes/fields, LiDAR IMU (gyro/accel/covariances/header), full
   LowState and optional robot_odom. Record integer source nanoseconds and PC
   monotonic receipt timestamps; bounded queues, sequence/counter/drop metrics.
   The probe's floating timestamps and synchronous disk writes are diagnostics,
   not the final synchronization implementation.
3. Verify time is relative seconds from scan start against upstream driver and
   live data. Keep original timestamps; do not replace with PC receipt time.
   Test delayed/reordered/missing IMU, duplicate clouds, clock jumps, and scan
   overlap. Buffer until IMU covers each scan end. No latest-only cloud dropping
   inside Point-LIO; explicit bounded backlog policy and invalidation instead.
4. Verify LiDAR IMU units and axes independently of LowState. Stationary norm
   10.4469 is about 6.5% above 9.81 if SI units: a mounting rotation alone cannot
   explain magnitude. Obtain driver/firmware conventions and compare multiple
   supported stationary poses. A single pose cannot identify full accelerometer
   bias/scale or all extrinsic axes. Do not normalize each acceleration sample
   to g, subtract observed gravity as bias, or blindly copy body gyro correction.
5. Verify required streams during sport-off in the next user-operated controlled
   test. Today's mcf-active sample cannot establish that requirement.

Exit: documented units/time/extrinsic contract; replay reproduces original cloud
bytes, fields and integer stamps exactly; gaps are reported and injected clock
faults fail closed. Unexplained acceleration scale/time mismatch blocks claims
of valid LIO accuracy, but permits labeled diagnostic runs.

## Stage B: DDS -> official Point-LIO -> base pose

Proposed directory: `deploy/parkour/lio/` (container, ROS package, pinned config,
launch, input/output adapters). Keep upstream external source under ignored
build/dependency storage rather than mixing files into the controller.

- DDS input adapter: rt/utlidar/cloud + rt/utlidar/imu -> ROS1
  /unilidar/cloud + /unilidar/imu. Preserve point time/ring/header and IMU values.
  Adapt shape/packing only when required and cover with roundtrip tests.
- Official baseline lidar_type=5, timestamp_unit=0, imu_en=true. Noise, scale,
  saturation and time-offset values require measured/official sensor evidence.
  Initial stationary initialization; retain default algorithm for first baseline.
- Confirm upstream output convention: camera_init -> aft_mapped represents the
  estimator IMU-body pose, not Go2 base_link. Test transforms on synthetic poses.
- Define T_A_B as mapping B coordinates into A. Compose:
  T_B_I = T_B_L * T_L_I; T_W_B = T_W_I * inverse(T_B_I).
  Official L1 T_L_I translation is [-0.007698,-0.014655,0.00667] m with parallel
  axes. Our measured T_B_L is in em_sidecar/go2_cloud.py, but alignment of Go2's
  published utlidar_imu to official L1 axes must be verified before composition.
  Do not use a LiDAR<->IMU translation as the base mounting translation.
- Output independent DDS `rt/parkour/lio/odom` plus validity/health status;
  source stamp, generation/reset ID, initialization/tracking status, backlog and
  latency. A finite pose or covariance alone is not a tracking-health guarantee.
  Never impersonate firmware rt/utlidar/robot_odom.

Exit: static replay initializes, finite continuous base pose, correct frame
composition, health/reset tests pass; report actual output rate and latency.

## Stage C: compare two estimators without motor control

Use one immutable input recording and fixed configs for both paths. Keep the
existing leg estimator as-is and run it using recorded LowState ticks. Align
initial world translation/yaw once (no scale fit or per-segment realignment).
Relate LowState ticks to LiDAR time with an explicit measured clock mapping;
quantify arrival jitter rather than declaring PC receive time hardware sync.

Trials, performed by the user when motion is needed:
- Static standing >=60 s, repeated three times.
- Stand -> down -> stand, dwell >=10 s per stable pose, repeat three times.
- Later, already-authorized controlled straight motion and return, known distance,
  yaw turn/return, and a loop. No autonomous movement for data collection.
- Known floor/step geometry; include sparse/occluded terrain and data-drop replay.

Report: position/yaw drift, height change, return residual, trajectory jumps,
tracking loss, valid time, gap distribution, CPU/memory and end-to-end latency.
robot_odom and mutual agreement are references, not ground truth. Use external
measured endpoints/yaw or external tracking for absolute accuracy; never use
Point-LIO's own map as the sole judge of Point-LIO quality.

Separate two comparisons: pose-source effect with the same raw clouds, then
complete mapping pipelines including LIO motion compensation. Label these
clearly: using LIO-deskewed clouds for both methods introduces LIO dependence.
Do not claim Point-LIO superiority solely from a static leg-estimator result
already measured at ~1.7 mm/min. Keep both if tradeoffs differ by motion regime.

## Stage D: elevation-map shadow integration

Current code touchpoints:
- tools/go2_sensor_bridge.py: run(), sensor_callback(), preceding_pose(), CLI.
- tools/go2_leg_pose.py: preserve leg mode and calibration behavior.
- em_sidecar/go2_cloud.py: preserve existing raw/base transform; no double transform.
- tools/replay_go2_base_scan.py: add repeatable source comparison/replay.
- tools/go2_frame_record.cpp: optional consolidated input recorder extension.

Add explicit `--odom lio`, retaining leg/robot behavior. Important: current
preceding_pose pairs PC arrival times with a 20 ms age limit. LIO is delayed
relative to cloud arrival; simply subscribing a new odom topic will not work.
Use sensor-stamp keyed bounded cloud/pose buffers and interpolate or associate
poses at the documented scan reference time; no unlimited extrapolation.
Keep LowState liveness as a separate check. On resets, clock changes or tracking
loss invalidate terrain, clear persistent map and require reinitialization.

Position AND orientation must come from one coherent estimator world frame.
For compensated clouds, specify output frame and reference time, then convert
once into base frame; preserve original per-point fields for any self filter.
Verify motion compensation isn't applied twice. Self-return filtering remains
an independent task; LIO does not make occluded floor observable.

Shadow comparison: separate map instances and outputs
`rt/parkour/scandots_leg_eval`, `rt/parkour/scandots_lio_eval`; never two publishers
on the production topic. go2_ctrl's normal LowState/ONNX/LowCmd path stays intact.

## Proposed acceptance criteria (engineering targets, not sensor guarantees)

- All input byte/field/time/frame roundtrip and synthetic transform tests pass.
- Zero silent clock resets/backlog losses; invalid LIO immediately invalidates
  its evaluation map. Explicit stale/reset/reinitialization tests.
- Same replay repeated produces matching results within documented tolerance.
- Initial candidate target: 60 s static xy drift <2 cm, yaw <0.5 degrees; return
  residual xy <2 cm, z <1 cm. Targets do not replace comparison with better leg
  performance or physical reference. Input anomaly must be resolved separately.
- Same observed floor cells before/after motion: median difference <1 cm,
  p95 <2 cm; exclude unknown/upper-bound-only cells, align physical sample sites,
  and report direct-observation history. Do not call map valid bits direct rays.
- Mapping sustained >=9.5 Hz with p95 interval <=120 ms and max <=150 ms after
  warmup on the existing desktop; report full end-to-end latency separately.
- Three paired trials of each motion; report both successes and failure cases.
  Choose primary source from measured moving accuracy and availability, not a
  presumed LIO advantage. No automatic source-switch fallback in this scope.

## Sources

- Official Unitree implementation: https://github.com/unitreerobotics/point_lio_unilidar
- Pinned L1 config: https://github.com/unitreerobotics/point_lio_unilidar/blob/18ed5976d8fab2bd8a5148c26a40692bd3c0dc91/config/unilidar_l1.yaml
- L1 geometry: https://github.com/unitreerobotics/unilidar_sdk#coordinate-system-definition
- Point-LIO timing requirements: https://github.com/hku-mars/Point-LIO
- Noetic lifecycle: https://discourse.ros.org/t/ros-noetic-end-of-life-may-31-2025/43160

Implementation status: input validation, lossless JSONL/DDS replay, isolated
official baseline, paired diagnostic comparison, and guarded deskewed map path
are implemented. See `lio/README.md` for commands and verified counts. Actual
base-frame calibration and acceleration validation remain blocked on evidence;
the map path deliberately rejects the current inconsistent nominal transform.
Hardware motion and live policy adoption remain separate checkpoints.

## Sport-off follow-up (2026-09-16)

Subscriber-only 20 s probe in `captures/lio_feasibility_sportoff_20260916/`.
CheckMode returned code 0 with empty name. LiDAR IMU 248.56 Hz (4973 samples),
raw cloud 15.398 Hz (308 frames), LowState 500.05 Hz. No IMU/cloud header
duplicates or regressions. robot_odom had zero samples in this window.
Joint angles indicate stationary down/folded posture, unlike prior standing
measurement: compare stream availability but do not attribute value changes
solely to sport mode. IMU acceleration mean [3.80021,-0.00128,9.80646],
mean-vector magnitude 10.517046 m/s^2 assuming SI. The acceleration discrepancy
persists sport-off. Both required LIO sensor streams remain available in this
test; IMU calibration/time validation remains open. No motion/mode command sent.
