# Go2 initial hardware inspection — 2026-09-15

## Scope

Read-only SSH inspection of Jetson and subscriber-only DDS probes on the PC.
No motor commands, motion/service transitions, remote edits, reboots, or clock
changes were performed. Local diagnostics and point-cloud decoding are the only
code changes. Robot identity/firmware and safety readiness are not certified.

## Network and hardware evidence

- PC robot interface: `enp42s0`, `192.168.123.99/24`, DDS domain 0.
- PC internet remains on `enx80691a73e263`.
- Jetson `eth0`: `192.168.123.18/24`, default gateway `192.168.123.1`.
- SSH device-tree model: `NVIDIA Orin NX Developer Kit`.
- OS: Ubuntu 20.04.5 LTS, aarch64, kernel `5.10.104-tegra`.
- `/etc/nv_tegra_release`: R35 revision 3.1. MemTotal: 15758116 kB.
- Jetson UTC clock reported February 1970. Boot journal wall-clock dates also
  vary. Use boot-relative journal timestamps for this inspection; do not equate
  Jetson wall time with sensor timestamps or the PC clock.

## Existing startup software (on Jetson)

`maum_ral_ros.service` is enabled and runs
`/home/raise/ral-go2-driver/launch_ral_auto.sh`, which execs
`./build/ral_go2_zmq_node`. It initially failed when eth0 was unavailable,
restarted, and logged ready with SDK2 SportClient about 21 seconds after boot.
`maum_ral_sio.service` is also enabled and runs the existing Socket.IO bridge.
Both report `Restart=on-failure`, restart delay 2 seconds.

Source inspected on Jetson:

- `/home/raise/ral-go2-driver/src/ral_go2_ros/ral_go2_zmq_node/src/robot_controller.cpp`
  lines 79–118: DDS/SportClient initialization and subscriptions; no explicit
  stand call in this initialization path.
- Same file, lines 122–124: `standUp()` invokes `RecoveryStand()`.
- `bridge_node.cpp` lines 180–186: received `stand_up` action dispatches that call.
- `robot_controller.cpp` lines 44–65 and 147–155: a received velocity command
  enables repeated Move calls at 20 Hz; initialization sets this flag false.

**Inference limit:** existing motion-capable software is present, but neither
the inspected startup path nor available boot logs prove it caused the reported
automatic rise. Built-in startup behavior and other command sources remain
unresolved. Absence of a log entry is not proof that no command was sent; the
deployed binary was not verified byte-for-byte against the source.

Other running services include grid_map, power_monitoring, video, and Unitree
upgrade services. `rms_odom.service` had over 200 restarts. Its `/var/log/grid.log`
trace shows an MQTT connection timeout in `RMS_odom.py:init_mqtt`. This is existing
application behavior; it does not establish a fault in direct DDS odometry and
was not changed.

## Direct DDS observation

Earlier lowstate probe: 2483 callbacks in 5 seconds (~497 Hz), battery 93%,
31.2 V, joint positions/velocities and IMU populated. Foot-force raw sample:
40, 40, 43, 41. `sn` and `version` were zero and cannot identify firmware.

Perception probe, 10 seconds (`tools/go2_perception_probe.cpp`):

| Topic | Evidence |
|---|---|
| rt/sportmodestate | 2950 callbacks, ~295.6 Hz, maximum callback gap 17.3 ms |
| rt/utlidar/cloud | 154 callbacks, ~15.4 Hz, maximum callback gap 75.4 ms |

Sport samples: mode=0, error_code=100, gait=0; position approximately
(-0.003, 0.006, 0.313), body_height approximately 0.322. **The meaning of
error_code=100 is unverified**; reception success is not a clean health verdict.
Rates/gaps are application callback measurements, not certified packet-loss or
real-time control measurements.

Cloud samples: frame `utlidar_lidar`, height=1, about 4200 points, little endian,
point_step=32, row_step=width*32. Fields:

| Field | Offset | Type |
|---|---:|---|
| x | 0 | FLOAT32 |
| y | 4 | FLOAT32 |
| z | 8 | FLOAT32 |
| intensity | 16 | FLOAT32 |
| ring | 20 | UINT16 |
| time | 24 | FLOAT32 |

Reported sampled frames had finite XYZ and structurally valid payloads. This
does not calibrate sensor orientation, extrinsics, units, or map accuracy.
Raw probe summary is saved locally at `/tmp/go2-perception.txt` (temporary).

## Sensor-to-policy incompatibility found

The old `em_sidecar/sidecar.py:on_cloud` assumed width contiguous XYZ triples
(12 bytes per point), ignoring point_step, fields, row_step, and endianness.
The actual cloud is 32 bytes per point. Its intensity/ring/time/padding would
therefore be interpreted as coordinates. Metadata-aware decoding is required
before using this stream to build policy observations.

Implemented `em_sidecar/pointcloud.py:decode_xyz` and integrated it into
`sidecar.py:on_cloud`. It respects field offsets/types, point and row strides,
and byte order; rejects malformed metadata/payloads and drops nonfinite points.
Seven unittest cases passed, covering the measured 32-byte layout, existing
12-byte simulator format, organized row padding, big-endian float64, nonfinite
points, empty frames, and malformed metadata. `git diff --check` passed.
This validates the decoder against constructed messages matching observed
metadata, not a full recorded-cloud-to-policy hardware replay.

Remaining mapping checks: sensor-to-base transform, odometry origin offset,
contact thresholds, timestamp alignment and handling ~15.4 Hz input versus
the training map update interval of 0.1 seconds. Current on_cloud processes
each cloud; a matching frame name alone does not validate the transform.

## ONNX offline check

CPU ONNX Runtime loaded policy.onnx with prop [1,53], scan [1,132], hist [1,530]
and actions [1,12]. Replayed 100 existing golden-trace frames, all finite,
maximum action absolute error 2.03e-6; allclose atol=1e-5, rtol=1e-4 passed.
Observed Python session inference median 0.577 ms, p95 0.683 ms, maximum
4.905 ms. This is offline inference, not the live C++ control loop or end-to-end
latency. No real-sensor closed-loop or shadow-policy validation is claimed.

## Next gate

Validate the corrected point decoder, record real cloud/pose snapshots and
confirm the resulting ground geometry before producing live policy inputs.
Resolve status code 100 and independent stop/recovery arrangements before
motor trials. Existing services were left running; do not start go2_ctrl as a
diagnostic, as it can release motion control and publishes LowCmd even in Passive.

## Read-only odometry dependency investigation

A later live SSH inspection and PC DDS built-in discovery identified the actual
endpoint graph without ReleaseMode, service changes, or application command
writers. Evidence is in `captures/odom_dependency/dds_discovery.json`.

DDS participant properties advertise all three processes on `192.168.123.161`
(hostname `Unitree`), distinct from Jetson `192.168.123.18`:

| Process | Observed incoming topics | Observed outgoing topics |
|---|---|---|
| `basic_service` (PID 974) | `rt/lowcmd` | `rt/lowstate` |
| `mcf_main` (PID 1782) | `rt/lowstate`, other control inputs | `rt/lowcmd`, `rt/sportmodestate` |
| `unitree_lidar_server` (PID 1465) | **`rt/sportmodestate`**, its raw cloud, configuration topics | `rt/utlidar/robot_odom`, `robot_pose`, `cloud_base`, `cloud_deskewed`, raw cloud, `imu` |

These are participant-advertised host/process properties and discovered DDS
endpoints, not an SSH process inspection of the internal .161 computer. The
actual lidar-server sport-state subscription establishes a communication
dependency. Together with previously measured nearly identical sport/robot_odom
poses, it argues against treating robot_odom as an independent LiDAR-IMU pose
estimate. Endpoint discovery does not reveal its internal computation or prove
what happens when ReleaseMode is called. In particular, whether mcf_main keeps
publishing valid sport state after motion-control release remains untested.

Jetson inspection found:

- `grid_map.service`: `/home/unitree/suda_v3/grid_map.py`, subscribes to
  `rt/uslam/cloud_map`; not the discovered robot_odom writer.
- `rms_odom.service`: `/home/unitree/hbt_slam/RMS_odom.py`, subscribes to
  `rt/uslam/localization/odom`; not the discovered robot_odom writer.
- `/home/unitree/domain_relay_utlidar_robot_odom.py`: ROS 2 domain relay,
  defaults domain 0 to 77. Source contains forwarding, not pose estimation;
  no matching relay process was found in the process listing.
- `/unitree/module/graph_pid_ws` has SLAM packages and `Odometer_service` has
  SVO-related installation files. Presence on disk does not establish that
  either is active or produces the observed odometry.
- No sport/lidar-server process with those names was seen on Jetson. The DDS
  process/address evidence instead points to the internal .161 computer.

`rt/utlidar/imu` is advertised as `sensor_msgs::msg::dds_::Imu_`, useful as a
potential independent LIO input. Data quality, clock alignment and extrinsics
were not tested in this investigation. No estimator implementation or source
code change was made. SSH session was closed after inspection.
