"""Offline 50 Hz replay of recorded Go2 state and 10 Hz terrain scans.

This diagnostic executes the ONNX policy but never imports DDS or publishes a
command.  Recorded LowState is open-loop: inferred actions are fed back only to
the policy's ``last_action`` observation and action-delay pipeline; they cannot
have affected the recorded robot motion.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import yaml


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_RUNTIME_CONFIG = ROOT.parent / "robots/go2/config/config.yaml"
NUM_PROP = 53
NUM_SCAN = 132
NUM_HIST = 530
NUM_JOINTS = 12
HIST_LEN = 10
POLICY_PERIOD_NS = 20_000_000


def read_jsonl(path: Path) -> tuple[list[dict], np.ndarray]:
    rows = [json.loads(line) for line in path.read_text().splitlines() if line.strip()]
    if not rows:
        raise ValueError(f"no messages in {path}")
    times = np.asarray([row["steady_ns"] for row in rows], dtype=np.int64)
    if np.any(np.diff(times) < 0):
        raise ValueError(f"non-monotonic timestamps in {path}")
    return rows, times


def preceding(times: np.ndarray, ticks: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Return causal latest indices and ages; an index can be -1."""
    indices = np.searchsorted(times, ticks, side="right").astype(np.int64) - 1
    ages_ms = np.full(len(ticks), np.nan, dtype=np.float64)
    valid = indices >= 0
    ages_ms[valid] = (ticks[valid] - times[indices[valid]]) * 1e-6
    return indices, ages_ms


def build_policy_schedule(
    low_times: np.ndarray,
    scan_times: np.ndarray,
    max_low_age_ms: float = 20.0,
    max_scan_age_ms: float = 500.0,
) -> dict[str, np.ndarray]:
    """Build 50 Hz ticks using only samples available at each tick.

    Ticks retain the 10 Hz map schedule's phase.  Scan timestamps are map
    *availability* times, not the source-cloud timestamps.
    """
    low_times = np.asarray(low_times, dtype=np.int64)
    scan_times = np.asarray(scan_times, dtype=np.int64)
    if not len(low_times) or not len(scan_times):
        raise ValueError("low and scan times must be non-empty")
    start = int(scan_times[0])
    end = min(int(low_times[-1]), int(scan_times[-1]) + int(max_scan_age_ms * 1e6))
    ticks = start + np.arange((end - start) // POLICY_PERIOD_NS + 1, dtype=np.int64) * POLICY_PERIOD_NS
    low_index, low_age_ms = preceding(low_times, ticks)
    scan_index, scan_age_ms = preceding(scan_times, ticks)
    usable = ((low_index >= 0) & (scan_index >= 0) &
              (low_age_ms <= max_low_age_ms) & (scan_age_ms <= max_scan_age_ms))
    return {
        "tick_steady_ns": ticks,
        "low_index": low_index,
        "low_age_ms": low_age_ms,
        "scan_index": scan_index,
        "scan_age_ms": scan_age_ms,
        "usable": usable,
    }


def euler_xyz_from_quat(quat_wxyz: np.ndarray) -> tuple[np.float32, np.float32, np.float32]:
    w, x, y, z = np.asarray(quat_wxyz, dtype=np.float32)
    roll = np.arctan2(np.float32(2) * (w * x + y * z),
                      np.float32(1) - np.float32(2) * (x * x + y * y))
    sin_pitch = np.float32(2) * (w * y - z * x)
    pitch = np.copysign(np.float32(np.pi / 2), sin_pitch) if abs(sin_pitch) >= 1 else np.arcsin(sin_pitch)
    yaw = np.arctan2(np.float32(2) * (w * z + x * y),
                     np.float32(1) - np.float32(2) * (y * y + z * z))
    return np.float32(roll), np.float32(pitch), np.float32(yaw)


def wrap_to_pi(value: float) -> np.float32:
    value = np.float32(value)
    pi = np.float32(np.pi)
    wrapped = np.fmod(value + pi, np.float32(2) * pi)
    if wrapped < 0:
        wrapped += np.float32(2) * pi
    if wrapped == 0 and value > 0:
        return pi
    return np.float32(wrapped - pi)


def build_prop(
    low: dict,
    default_joint_pos: np.ndarray,
    il_to_sdk: np.ndarray,
    il_foot_to_sdk: np.ndarray,
    last_action: np.ndarray,
    previous_contact: np.ndarray,
    contact_threshold: float,
    cmd_vx: float,
    delta_yaw: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Mirror ``ObservationBuilder`` and State_Parkour LowState extraction."""
    prop = np.zeros(NUM_PROP, dtype=np.float32)
    gyro = np.asarray(low["imu_state"]["gyroscope"], dtype=np.float32)
    quat = np.asarray(low["imu_state"]["quaternion"], dtype=np.float32)
    roll, pitch, _ = euler_xyz_from_quat(quat)
    prop[0:3] = gyro * np.float32(0.25)
    prop[3], prop[4] = wrap_to_pi(roll), wrap_to_pi(pitch)
    prop[6:8] = np.float32(delta_yaw)
    prop[10] = np.float32(cmd_vx)
    prop[11] = 1.0
    motors = low["motor_state"]
    prop[13:25] = np.asarray([motors[int(i)]["q"] for i in il_to_sdk], dtype=np.float32) - default_joint_pos
    prop[25:37] = np.asarray([motors[int(i)]["dq"] for i in il_to_sdk], dtype=np.float32) * np.float32(0.05)
    prop[37:49] = np.asarray(last_action, dtype=np.float32)
    forces = np.asarray(low["foot_force"], dtype=np.float32)[il_foot_to_sdk]
    current_contact = forces > np.float32(contact_threshold)
    filtered_contact = current_contact | previous_contact
    prop[49:53] = filtered_contact.astype(np.float32) - np.float32(0.5)
    return prop, current_contact


class History:
    def __init__(self) -> None:
        self.frames: list[np.ndarray] = []

    @staticmethod
    def masked(prop: np.ndarray) -> np.ndarray:
        frame = np.asarray(prop, dtype=np.float32).copy()
        frame[6:8] = 0
        return frame

    def reset(self) -> None:
        self.frames = []

    def prime(self, prop: np.ndarray) -> None:
        frame = self.masked(prop)
        self.frames = [frame.copy() for _ in range(HIST_LEN)]

    def value(self) -> np.ndarray:
        if len(self.frames) != HIST_LEN:
            raise RuntimeError("history is not primed")
        return np.concatenate(self.frames)

    def push(self, prop: np.ndarray) -> None:
        self.frames = self.frames[1:] + [self.masked(prop)]


class ActionDelay:
    def __init__(self, delay_steps: int, clip: tuple[float, float], scale: float,
                 default_joint_pos: np.ndarray) -> None:
        self.delay_steps = delay_steps
        self.clip = clip
        self.scale = np.float32(scale)
        self.default = np.asarray(default_joint_pos, dtype=np.float32)
        self.queue: list[np.ndarray] = []
        self.reset()

    def reset(self) -> None:
        self.queue = [np.zeros(NUM_JOINTS, dtype=np.float32)
                      for _ in range(max(2, self.delay_steps + 1))]

    def last_raw(self) -> np.ndarray:
        return self.queue[-1]

    def push(self, raw: np.ndarray) -> np.ndarray:
        self.queue = self.queue[1:] + [np.asarray(raw, dtype=np.float32).copy()]
        delayed = self.queue[max(0, len(self.queue) - 1 - self.delay_steps)]
        return np.clip(delayed, *self.clip) * self.scale + self.default


def validate_golden(golden_path: Path, cfg: dict) -> dict[str, float | int]:
    """Check Python observation/history semantics against the IsaacLab trace."""
    with np.load(golden_path) as data:
        prop_ref = data["prop"][:, 0]
        action_ref = data["actions"][:, 0]
        hist_ref = data["hist"][:, 0]
        joint_pos = data["joint_pos"][:, 0]
        joint_vel = data["joint_vel"][:, 0]
        gyro = data["root_ang_vel_b"][:, 0]
        quat = data["root_quat_w"][:, 0]
        contact_now = data["contact_now"][:, 0]
        contact_prev = data["contact_prev"][:, 0]
        props = []
        default = np.asarray(cfg["default_joint_pos"]["isaaclab"], dtype=np.float32)
        for t in range(len(prop_ref)):
            fake = {
                "imu_state": {"gyroscope": gyro[t], "quaternion": quat[t]},
                "motor_state": [{"q": joint_pos[t, i], "dq": joint_vel[t, i]} for i in range(12)],
                "foot_force": [0, 0, 0, 0],
            }
            last = prop_ref[0, 37:49] if t == 0 else action_ref[t - 1]
            # Golden forces are vectors; inject the exact filtered Boolean into
            # the scalar-force adapter using identity ordering.
            now = np.linalg.norm(contact_now[t], axis=1) > 2.0
            prev = np.linalg.norm(contact_prev[t], axis=1) > 2.0
            fake["foot_force"] = (now.astype(np.float32) * 3.0).tolist()
            p, _ = build_prop(fake, default, np.arange(12), np.arange(4), last,
                              prev, 2.0, prop_ref[t, 10], prop_ref[t, 6])
            p[7] = prop_ref[t, 7]
            props.append(p)
        props = np.asarray(props)
        prop_error = float(np.max(np.abs(props - prop_ref)))

        history = History()
        hist_error = 0.0
        compared = 0
        for t, prop in enumerate(prop_ref):
            if t == 0:
                history.prime(prop)
            else:
                if t >= 11:
                    hist_error = max(hist_error, float(np.max(np.abs(history.value() - hist_ref[t]))))
                    compared += 1
                history.push(prop)
    if prop_error > 3e-6 or hist_error != 0.0:
        raise AssertionError(f"golden validation failed: prop={prop_error}, hist={hist_error}")
    return {"prop_max_abs_error": prop_error, "history_max_abs_error": hist_error,
            "history_frames_compared": compared}


def replay(low_path: Path, scan_path: Path, contract_path: Path, policy_path: Path,
           golden_path: Path, cmd_vx: float = 0.3, action_delay_steps: int = 0,
           contact_threshold: float = 2.0, runtime_config_path: Path | None = None,
           max_low_age_ms: float = 20.0,
           max_scan_age_ms: float = 500.0) -> tuple[dict[str, np.ndarray], dict]:
    import onnxruntime as ort

    cfg = yaml.safe_load(contract_path.read_text())
    golden = validate_golden(golden_path, cfg)
    lows, low_times = read_jsonl(low_path)
    with np.load(scan_path) as data:
        accepted = data["accepted"].astype(bool)
        scan_times = data["tick_steady_ns"][accepted].astype(np.int64)
        scans = data["scan"][accepted].astype(np.float32)
    schedule = build_policy_schedule(low_times, scan_times, max_low_age_ms, max_scan_age_ms)

    default = np.asarray(cfg["default_joint_pos"]["isaaclab"], dtype=np.float32)
    il_to_sdk = np.asarray(cfg["index_maps"]["il_to_sdk"], dtype=np.int64)
    feet = np.asarray(cfg["index_maps"]["il_foot_to_sdk"], dtype=np.int64)
    action_cfg = cfg["action"]
    history = History()
    delay = ActionDelay(action_delay_steps, tuple(action_cfg["clip"]),
                        float(action_cfg["scale"]), default)
    session = ort.InferenceSession(str(policy_path), providers=["CPUExecutionProvider"])

    kept_ticks, kept_low, kept_scan = [], [], []
    low_ages, scan_ages, props, hists, used_scans = [], [], [], [], []
    actions, q_targets, contacts = [], [], []
    previous_contact = np.zeros(4, dtype=bool)
    target_yaw = None
    resets = 0
    in_gap = False
    for i, usable in enumerate(schedule["usable"]):
        if not usable:
            if history.frames and not in_gap:
                history.reset(); delay.reset(); previous_contact[:] = False; resets += 1
                target_yaw = None
            in_gap = True
            continue
        in_gap = False
        low = lows[int(schedule["low_index"][i])]
        _, _, yaw = euler_xyz_from_quat(low["imu_state"]["quaternion"])
        if target_yaw is None:
            target_yaw = yaw
        # Neutral joystick: target yaw stays at the initial yaw.  State_Parkour
        # updates the measured yaw each policy step and scales the error by 1.5.
        delta_yaw = np.float32(1.5) * wrap_to_pi(float(target_yaw - yaw))
        prop, previous_contact = build_prop(
            low, default, il_to_sdk, feet, delay.last_raw(), previous_contact,
            contact_threshold, cmd_vx, delta_yaw,
        )
        if not history.frames:
            history.prime(prop)
        hist = history.value().copy()
        scan = scans[int(schedule["scan_index"][i])]
        if not (np.isfinite(prop).all() and np.isfinite(hist).all() and
                np.isfinite(scan).all()):
            raise ValueError(f"non-finite policy input at tick {int(schedule['tick_steady_ns'][i])}")
        raw = session.run(None, {
            "prop": prop[None, :], "scan": scan[None, :], "hist": hist[None, :]
        })[0][0].astype(np.float32)
        if raw.shape != (NUM_JOINTS,) or not np.isfinite(raw).all():
            raise ValueError(f"invalid ONNX output at tick {int(schedule['tick_steady_ns'][i])}")
        q_target = delay.push(raw)
        if not np.isfinite(q_target).all():
            raise ValueError(f"non-finite q target at tick {int(schedule['tick_steady_ns'][i])}")
        history.push(prop)

        kept_ticks.append(schedule["tick_steady_ns"][i])
        kept_low.append(schedule["low_index"][i]); kept_scan.append(schedule["scan_index"][i])
        low_ages.append(schedule["low_age_ms"][i]); scan_ages.append(schedule["scan_age_ms"][i])
        props.append(prop); hists.append(hist); used_scans.append(scan)
        actions.append(raw); q_targets.append(q_target); contacts.append(previous_contact.copy())

    if not kept_ticks:
        raise ValueError("no usable 50 Hz policy ticks")
    arrays = {
        "tick_steady_ns": np.asarray(kept_ticks, dtype=np.int64),
        "lowstate_index": np.asarray(kept_low, dtype=np.int64),
        "scan_index": np.asarray(kept_scan, dtype=np.int64),
        "lowstate_age_ms": np.asarray(low_ages, dtype=np.float64),
        "scan_age_ms": np.asarray(scan_ages, dtype=np.float64),
        "prop": np.asarray(props, dtype=np.float32),
        "hist": np.asarray(hists, dtype=np.float32),
        "scan": np.asarray(used_scans, dtype=np.float32),
        "raw_action": np.asarray(actions, dtype=np.float32),
        "q_target": np.asarray(q_targets, dtype=np.float32),
        "contact": np.asarray(contacts, dtype=bool),
    }
    raw = arrays["raw_action"]
    clipped = (raw < float(action_cfg["clip"][0])) | (raw > float(action_cfg["clip"][1]))
    rejected = ~schedule["usable"]
    summary = {
        "scope": "offline open-loop recorded LowState + held 10 Hz scan -> 50 Hz ONNX inference",
        "policy_ticks": int(len(arrays["tick_steady_ns"])),
        "duration_s": float((arrays["tick_steady_ns"][-1] - arrays["tick_steady_ns"][0]) * 1e-9),
        "command_assumption": {
            "cmd_vx_mps": float(cmd_vx),
            "source": "explicit replay argument; CLI resolves runtime minimum or override",
            "heading": "neutral joystick holds first replay yaw; delta=1.5*wrap(target_yaw-yaw)",
        },
        "action_delay": {
            "training_contract_steps": int(action_cfg["delay_steps"]),
            "effective_runtime_steps": int(action_delay_steps),
            "source": str(runtime_config_path) if runtime_config_path else "explicit replay argument",
            "note": "physical DDS/actuator latency is not present in this offline replay",
        },
        "causality": {
            "lowstate": "latest LowState receipt <= policy tick",
            "scan": "latest accepted map processing tick <= policy tick; zero-order hold; no future scan",
            "lowstate_age_ms": stats(arrays["lowstate_age_ms"]),
            "scan_age_ms": stats(arrays["scan_age_ms"]),
        },
        "rejected_schedule_ticks": int(rejected.sum()),
        "offline_gap_resets": resets,
        "guard_semantics": {
            "scan_age_limit_ms": max_scan_age_ms,
            "threshold_equals_runtime_scan_timeout": True,
            "scan_availability_is_synthetic_not_dds_receipt": True,
            "lowstate_age_limit_ms": max_low_age_ms,
            "lowstate_limit_is_offline_diagnostic_only": True,
            "runtime_lowstate_timeout_ms": 1000.0,
            "runtime_lowstate_timeout_source": "FSMState LowState isTimeout -> Passive; Subscription default",
            "offline_lowstate_limit_is_stricter_than_runtime": True,
        },
        "observation": {
            "prop_shape": list(arrays["prop"].shape), "hist_shape": list(arrays["hist"].shape),
            "scan_shape": list(arrays["scan"].shape), "all_finite": bool(all(
                np.isfinite(arrays[k]).all() for k in ("prop", "hist", "scan"))),
            "history_excludes_current_prop": True, "history_heading_indices_masked": True,
            "contact_threshold_raw_units": float(contact_threshold),
            "contact_calibration_validated": False,
        },
        "onnx": {
            "providers": session.get_providers(), "action_shape": list(raw.shape),
            "all_finite": bool(np.isfinite(raw).all()), "raw_action": stats(raw),
            "values_outside_action_clip": int(clipped.sum()),
            "fraction_outside_action_clip": float(clipped.mean()),
            "q_target_rad": stats(arrays["q_target"]),
        },
        "golden_validation": golden,
        "interpretation_limits": [
            "Recorded robot state did not respond to inferred actions; this is not a closed-loop stability test.",
            "The robot was standing; a nonzero command requests unrecorded motion, and zero is outside the current runtime command range.",
            "The approximately 35 mm map/body height mismatch and self-filter validation remain unresolved.",
            "The scalar real-robot foot-force threshold is uncalibrated.",
            "Map computation latency after its synthetic processing tick is absent from the recording.",
        ],
        "publishes_commands": False,
        "live_readiness": False,
    }
    return arrays, summary


def stats(values: np.ndarray) -> dict[str, float]:
    v = np.asarray(values, dtype=np.float64)
    return {"min": float(v.min()), "median": float(np.median(v)),
            "max": float(v.max()), "mean": float(v.mean())}


def plot_replay(arrays: dict[str, np.ndarray], output: Path) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    t = (arrays["tick_steady_ns"] - arrays["tick_steady_ns"][0]) * 1e-9
    fig, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True, constrained_layout=True)
    axes[0].plot(t, arrays["raw_action"], linewidth=0.7, alpha=0.75)
    axes[0].axhline(4.8, color="r", linestyle="--", linewidth=0.8)
    axes[0].axhline(-4.8, color="r", linestyle="--", linewidth=0.8)
    axes[0].set_ylabel("raw action"); axes[0].set_title("Offline policy replay (12 joints)")
    axes[1].plot(t, arrays["q_target"], linewidth=0.7, alpha=0.75)
    axes[1].set_ylabel("effective q target [rad]")
    axes[2].step(t, arrays["scan_age_ms"], where="post", label="held scan age")
    axes[2].plot(t, arrays["lowstate_age_ms"], label="LowState age", linewidth=0.8)
    axes[2].set_ylabel("causal sample age [ms]"); axes[2].set_xlabel("replay time [s]")
    axes[2].legend()
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=160)
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--lowstate", type=Path, required=True)
    parser.add_argument("--scan-replay", type=Path, required=True)
    parser.add_argument("--contract", type=Path, default=ROOT / "contract/deploy.yaml")
    parser.add_argument("--policy", type=Path, default=ROOT / "contract/policy.onnx")
    parser.add_argument("--golden", type=Path, default=ROOT / "contract/golden_trace.npz")
    parser.add_argument("--runtime-config", type=Path, default=DEFAULT_RUNTIME_CONFIG)
    parser.add_argument("--cmd-vx", type=float,
                        help="override neutral runtime command (default: runtime cmd_vx_min)")
    parser.add_argument("--action-delay-steps", type=int,
                        help="override effective runtime action delay")
    parser.add_argument("--skip-plot", action="store_true",
                        help="write NPZ/JSON without matplotlib (plot can be generated in another environment)")
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    runtime = yaml.safe_load(args.runtime_config.read_text())["FSM"]["Parkour"]
    cmd_vx = float(runtime["cmd_vx_min"]) if args.cmd_vx is None else args.cmd_vx
    contract = yaml.safe_load(args.contract.read_text())
    delay_steps = int(runtime.get("action_delay_override", -1)) \
        if args.action_delay_steps is None else args.action_delay_steps
    if args.action_delay_steps is not None and delay_steps < 0:
        parser.error("--action-delay-steps must be nonnegative")
    if delay_steps < 0:
        delay_steps = int(contract["action"]["delay_steps"])
    arrays, summary = replay(
        args.lowstate, args.scan_replay, args.contract, args.policy, args.golden,
        cmd_vx, delay_steps, float(runtime["contact_threshold"]), args.runtime_config,
        max_scan_age_ms=float(runtime.get("scandots_timeout", 0.5)) * 1000,
    )
    summary["command_assumption"]["source"] = (
        "runtime config cmd_vx_min (neutral joystick)" if args.cmd_vx is None
        else "CLI --cmd-vx override"
    )
    summary["action_delay"]["source"] = (
        "runtime config override, falling back to training contract" if args.action_delay_steps is None
        else "CLI --action-delay-steps override"
    )
    args.output_dir.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(args.output_dir / "replay.npz", **arrays)
    (args.output_dir / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    if not args.skip_plot:
        plot_replay(arrays, args.output_dir / "policy_replay.png")
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
