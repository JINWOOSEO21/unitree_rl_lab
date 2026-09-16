# Go2 Point-LIO: independent evaluation path

Status: runnable **diagnostic** pipeline. It never sends motor commands or sport
RPCs. Existing `go2_ctrl` and default `--odom leg` behavior are unchanged.

## Verified and unresolved

Official Unitree Point-LIO commit
`18ed5976d8fab2bd8a5148c26a40692bd3c0dc91` builds in the pinned ROS Noetic container.
Actual Go2 recorded cloud + LiDAR IMU have produced Point-LIO odometry through
this adapter. Input gyro/acceleration are preserved; no guessed bias/scale fix.

**Do not treat the resulting nominal base pose as calibrated.** In the first
standing replay, composing the official L1 IMU mounting convention with Go2's
observed raw-cloud/base transform produced a base gravity direction inconsistent
with LowState. The host compares gravity directions (yaw independent); mismatch
>0.35 rad marks the pose invalid and blocks evaluation maps. A matching gravity
direction would still not prove yaw or translation extrinsics, nor tracking
quality. The LiDAR accelerometer's stationary norm ~10.45–10.52 also remains
unexplained. Raw LIO output is retained even when base pose is invalid.

The pipeline can therefore be built, replayed and diagnosed now. Physical
accuracy comparisons and walking-map adoption still require sensor/frame
validation. `comparison.json` explicitly labels residuals as diagnostics, not
accuracy against ground truth.

## Architecture

```
Host env_isaaclab: SDK2 Python subscribers (cloud, LiDAR IMU, LowState)
    | preserved JSON/base64 sensor records over loopback TCP :17654
    v
Docker: ROS1 gateway -> pinned official Point-LIO -> IMU-body odometry
    | raw odometry over same TCP connection
    v
Host: nominal base transform + gravity consistency check + JSONL results
    | optional diagnostic-only rt/parkour/lio/pose (String_)
    v
optional go2_sensor_bridge --odom lio -> per-point deskew -> evaluation map
```

ROS runs only inside the container. No ROS installation or dependency changes
are needed in conda. A new container/estimator is required for every run; do not
reuse an initialized map for another recording. ROS master :11319 and gateway
:17654 bind to loopback; run only one evaluation container at a time.

## Build (from deploy/parkour)

```bash
cd /home/seo-jinwoo/workspace/codes/unitree_rl_lab/deploy/parkour
docker build -t go2-point-lio:baseline -f lio/container/Dockerfile lio
```

The base image digest and source commit are pinned. `/installed-packages.txt`
and `/point_lio_revision` in the image record the resolved build environment;
apt package versions are recorded, not fully locked to a snapshot. Host SDK/
conda dependencies remain unchanged. Noetic is EOL; this is an isolated upstream
comparison baseline, not a migration of the host system.

## Replay the saved stationary sample (no robot needed)

Terminal 1:

```bash
docker run --rm --network host --name go2-lio-eval go2-point-lio:baseline
```

Wait for `gateway listening 127.0.0.1:17654`. Terminal 2:

```bash
cd /home/seo-jinwoo/workspace/codes/unitree_rl_lab/deploy/parkour
conda activate env_isaaclab
python -m lio.host --replay captures/lio_feasibility_20260916 \
  --output "captures/lio_replay_$(date +%Y%m%dT%H%M%S)"
```

Existing feasibility captures contain float timestamps and no joint dq. The
legacy importer flags that limitation and cannot recover lost timestamp bits.
New captures preserve integer source nanoseconds, original cloud bytes/fields,
IMU covariances, full joint q/dq and LowState tick. Replay defaults to real time;
`--rate 0.5` is available for slower offline processing. No replay DDS publication.

## Live recording and comparison (subscriber-only)

Start a fresh container in Terminal 1 as above. Terminal 2:

```bash
python -m lio.host --interface enp42s0 --duration 60 \
  --output "captures/lio_live_$(date +%Y%m%dT%H%M%S)"
```

This runs Point-LIO and the existing leg estimator on the same incoming events,
recording all inputs and outputs. It does not move the robot. Supported steady
standing for the initial 10 s is needed by the leg estimator; folded/unsupported
robot data may initialize LIO but correctly leaves leg odometry invalid.
Duration is bounded (default 60, max 300 s). Sensor rate/drop warnings are saved.

Files in the selected output directory:

- `input.jsonl`: replayable raw cloud (base64), IMU, LowState; can be large.
- `lio.jsonl`: original IMU-body pose, nominal base pose, validity diagnostics,
  generation ID, gateway completion report.
- `leg.jsonl`: existing leg estimator output, approximate mapping of LowState
  receipt to LiDAR source clock. This clock mapping is not hardware sync.
- `metadata.json`, `summary.json`: source, revision, counts, gap/duplicate report,
  trailing IMU coverage and unprocessed final scan count.

To compare after the run:

```bash
python -m lio.compare captures/YOUR_OUTPUT_DIRECTORY
```

This produces `comparison.json` and `comparison.npz`, with one initial
translation/yaw alignment only. No scale fit. No ground truth is implied. A
failed leg initialization is reported instead of substituting zero poses.

## Optional LIO elevation-map evaluation

Add `--publish-odom` to the **live** host command. This publishes only
`rt/parkour/lio/pose`; no firmware topic is overwritten. Start another terminal:

```bash
python tools/go2_sensor_bridge.py --interface enp42s0 --odom lio \
  --publish-scandots --scandots-topic rt/parkour/scandots_lio_eval \
  --duration 0 --summary-only
```

Production `rt/parkour/scandots` publication is rejected in LIO mode. Current
uncalibrated/axis-inconsistent poses intentionally produce no valid map.
The map consumes pose history by source timestamp, requires a bracket for each
point, compensates raw points into scan-end base frame, and performs no pose
extrapolation. The host-frame assumption must be resolved before trusting it.
Generation changes invalidate output and clear the map; clock regressions fail
closed. Missing/stale LIO or LowState invalidates output. No automatic leg fallback.

For a separate leg evaluation map use existing `--odom leg` with
`--scandots-topic rt/parkour/scandots_leg_eval`; never give both methods the same
topic. Such leg/raw and LIO/deskewed maps compare complete pipelines, not only
the pose estimator. Keep that distinction in any performance report.

## Checks

```bash
python -m unittest discover -s lio/tests
python -m unittest discover -s tools/tests
```

Tests cover input layout/time corruption, big-endian and row padding, duplicate/
regressing clocks, nominal transform roundtrip, gravity inversion, delayed pose
brackets, resets, per-point translation deskew, and production-topic rejection.
Real replay is an additional integration check, not a calibration certificate.

The final gateway report does not invent trailing IMU or silently claim all
frames were processed: scan ends beyond IMU coverage/output time are counted.
A finite odometry stream alone is not proof that Point-LIO is tracking correctly.

## Implementation verification — 2026-09-16

- Recorded standing replay: 305 Point-LIO poses; paired diagnostic comparison
  generated, but nominal frame inconsistency prevents an accuracy conclusion.
- Final 20 s live acquisition (`captures/lio_live_final_01`): 308 clouds,
  5,028 IMU samples, 10,118 LowState samples, 304 Point-LIO poses. Acquisition
  keeps IMU/LowState running for 0.25 s after the cloud deadline; final scan
  coverage passed with zero scans missing trailing IMU or final output.
- Two IMU source-clock gaps (22.95 and 27.11 ms) were reported, not hidden.
- Nominal base gravity disagreement was 143.05–148.02 degrees. All base poses
  were invalidated; the separate map probe emitted zero scans and zero fatal
  errors, with the frame fault and host shutdown explicitly reported.
- Live leg initialization did not complete, so the live paired comparison
  correctly refused to produce statistics. Do not infer relative performance
  from this acquisition. A supported stationary initialization is still needed.
- 13 new tests and 70 existing tools tests passed. No motor or sport RPC was
  sent. Evaluation processes stopped after bounded acquisition.

Next checkpoint: establish the actual LiDAR-IMU/body frame relationship and
explain the accelerometer magnitude, then repeat stationary paired validation
before any motion trial or policy integration.
