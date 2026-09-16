"""Audit offline policy targets without importing DDS or sending commands."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import yaml


ROOT = Path(__file__).resolve().parents[1]


def apply_action_pipeline(raw: np.ndarray, delay_steps: int, clip: tuple[float, float],
                          scale: float, default: np.ndarray) -> np.ndarray:
    """Reproduce ActionPipeline: delay, clip, scale, then default offset."""
    raw = np.asarray(raw, dtype=np.float64)
    default = np.asarray(default, dtype=np.float64)
    if raw.ndim != 2 or raw.shape[1] != len(default):
        raise ValueError("raw actions and default joint positions have incompatible shapes")
    if delay_steps < 0:
        raise ValueError("delay_steps must be non-negative")
    delayed = np.zeros_like(raw)
    if delay_steps == 0:
        delayed[:] = raw
    elif delay_steps < len(raw):
        delayed[delay_steps:] = raw[:-delay_steps]
    return np.clip(delayed, clip[0], clip[1]) * scale + default


def il_to_sdk(values: np.ndarray, mapping: np.ndarray) -> np.ndarray:
    """Place IsaacLab-ordered values at their SDK motor indices."""
    values = np.asarray(values)
    mapping = np.asarray(mapping, dtype=np.int64)
    if values.shape[-1] != len(mapping) or sorted(mapping.tolist()) != list(range(len(mapping))):
        raise ValueError("il_to_sdk must be a permutation matching the final dimension")
    out = np.empty_like(values)
    out[..., mapping] = values
    return out


def load_urdf_limits(path: Path, joint_names: list[str]) -> dict[str, dict[str, float]]:
    root = ET.parse(path).getroot()
    found: dict[str, dict[str, float]] = {}
    wanted = set(joint_names)
    for joint in root.findall("joint"):
        name = joint.attrib.get("name", "")
        if name not in wanted:
            continue
        limit = joint.find("limit")
        if limit is None:
            raise ValueError(f"URDF joint {name} has no limit")
        found[name] = {key: float(limit.attrib[key]) for key in ("lower", "upper", "velocity", "effort")}
    missing = wanted - set(found)
    if missing:
        raise ValueError(f"URDF is missing policy joints: {sorted(missing)}")
    return found


def audit(replay_path: Path, contract_path: Path, runtime_path: Path,
          urdf_path: Path) -> dict:
    contract = yaml.safe_load(contract_path.read_text())
    runtime = yaml.safe_load(runtime_path.read_text())["FSM"]["Parkour"]
    replay = np.load(replay_path)
    raw = np.asarray(replay["raw_action"], dtype=np.float64)
    recorded_target = np.asarray(replay["q_target"], dtype=np.float64)
    ticks = np.asarray(replay["tick_steady_ns"], dtype=np.int64)
    prop = np.asarray(replay["prop"], dtype=np.float64)
    if len(raw) < 2 or len(ticks) != len(raw) or prop.shape != (len(raw), 53):
        raise ValueError("replay must contain at least two aligned policy ticks")
    if raw.shape != (len(ticks), 12) or recorded_target.shape != raw.shape:
        raise ValueError("actions and targets must have shape (ticks, 12)")
    if not all(np.isfinite(v).all() for v in (raw, recorded_target, prop)):
        raise ValueError("non-finite action, target, or observation")

    names_il = list(contract["joint_names"]["isaaclab"])
    names_sdk = list(contract["joint_names"]["sdk"])
    mapping = np.asarray(contract["index_maps"]["il_to_sdk"], dtype=np.int64)
    mapped_names = [None] * len(names_il)
    for il_index, sdk_index in enumerate(mapping):
        mapped_names[int(sdk_index)] = names_il[il_index]
    mapping_matches_names = mapped_names == names_sdk

    action = contract["action"]
    default = np.asarray(contract["default_joint_pos"]["isaaclab"], dtype=np.float64)
    contract_delay = int(action["delay_steps"])
    override = int(runtime.get("action_delay_override", -1))
    effective_delay = override if override >= 0 else contract_delay
    clip = tuple(float(x) for x in action["clip"])
    scale = float(action["scale"])
    effective_target = apply_action_pipeline(raw, effective_delay, clip, scale, default)
    contract_target = apply_action_pipeline(raw, contract_delay, clip, scale, default)
    replay_error = np.abs(effective_target - recorded_target)
    sdk_target = il_to_sdk(effective_target, mapping)
    sdk_roundtrip_error = np.abs(sdk_target[:, mapping] - effective_target)

    limits_by_name = load_urdf_limits(urdf_path, names_il)
    lower = np.asarray([limits_by_name[n]["lower"] for n in names_il])
    upper = np.asarray([limits_by_name[n]["upper"] for n in names_il])
    velocity = np.asarray([limits_by_name[n]["velocity"] for n in names_il])
    urdf_effort = np.asarray([limits_by_name[n]["effort"] for n in names_il])
    below = effective_target < lower
    above = effective_target > upper
    position_violation = below | above

    dt = np.diff(ticks).astype(np.float64) * 1e-9
    if np.any(dt <= 0):
        raise ValueError("policy ticks are not strictly increasing")
    delta = np.diff(effective_target, axis=0)
    target_rate = np.abs(delta) / dt[:, None]
    rate_exceeds_urdf = target_rate > velocity[None, :]

    current_q = prop[:, 13:25] + default
    initial_jump = effective_target[0] - current_q[0]
    actuator = contract["actuator"]
    kp = np.asarray(actuator["stiffness"], dtype=np.float64)
    kd = np.asarray(actuator["damping"], dtype=np.float64)
    effort = np.asarray(actuator["effort_limit"], dtype=np.float64)

    joints = []
    for j, name in enumerate(names_il):
        joint_rate = target_rate[:, j]
        joint_delta = np.abs(delta[:, j])
        joints.append({
            "isaaclab_index": j,
            "sdk_index": int(mapping[j]),
            "name": name,
            "urdf": limits_by_name[name],
            "controller": {"kp": float(kp[j]), "kd": float(kd[j]),
                           "contract_effort_limit": float(effort[j])},
            "target_rad": {"min": float(effective_target[:, j].min()),
                           "max": float(effective_target[:, j].max())},
            "position_limit_violations": int(position_violation[:, j].sum()),
            "max_position_limit_excess_rad": float(max(
                np.max(lower[j] - effective_target[:, j]),
                np.max(effective_target[:, j] - upper[j]), 0.0)),
            "step_change": {"max_abs_rad": float(joint_delta.max()),
                            "max_abs_rate_rad_s": float(joint_rate.max()),
                            "intervals_above_urdf_velocity": int(rate_exceeds_urdf[:, j].sum())},
            "initial": {"recorded_q_rad": float(current_q[0, j]),
                        "target_rad": float(effective_target[0, j]),
                        "target_minus_recorded_q_rad": float(initial_jump[j])},
        })

    return {
        "scope": "offline target audit; no DDS import and no command publication",
        "inputs": {"ticks": int(len(raw)), "replay": str(replay_path),
                   "contract": str(contract_path), "runtime_config": str(runtime_path),
                   "urdf": str(urdf_path)},
        "mapping": {
            "il_to_sdk": mapping.tolist(),
            "sdk_names_from_mapping": mapped_names,
            "configured_sdk_names": names_sdk,
            "matches_joint_names": mapping_matches_names,
            "sdk_roundtrip_max_abs_error_rad": float(sdk_roundtrip_error.max()),
            "sdk_targets": [
                {"sdk_index": i, "name": names_sdk[i],
                 "min_rad": float(sdk_target[:, i].min()),
                 "max_rad": float(sdk_target[:, i].max())}
                for i in range(len(names_sdk))
            ],
            "controller_write": "State_Parkour::run writes q[i] to motor_cmd[il_to_sdk[i]].q",
        },
        "pipeline": {
            "training_contract_delay_steps": contract_delay,
            "runtime_override": override,
            "effective_delay_steps": effective_delay,
            "clip": list(clip), "scale": scale, "uses_default_offset": bool(action["use_default_offset"]),
            "reconstructed_replay_max_abs_error_rad": float(replay_error.max()),
            "contract_vs_runtime_target_max_abs_difference_rad": float(np.abs(contract_target - effective_target).max()),
            "raw_values_outside_clip": int(((raw < clip[0]) | (raw > clip[1])).sum()),
        },
        "aggregate": {
            "position_limit_violations": int(position_violation.sum()),
            "ticks_with_any_position_limit_violation": int(position_violation.any(axis=1).sum()),
            "target_intervals": int(len(delta)),
            "target_rate_samples_above_urdf_velocity": int(rate_exceeds_urdf.sum()),
            "intervals_with_any_target_rate_above_urdf_velocity": int(rate_exceeds_urdf.any(axis=1).sum()),
            "max_abs_target_step_rad": float(np.abs(delta).max()),
            "max_abs_target_rate_rad_s": float(target_rate.max()),
            "max_abs_initial_target_jump_rad": float(np.abs(initial_jump).max()),
        },
        "gains_and_limits": {
            "parkour_state_source": "contract actuator stiffness/damping; State_Parkour::enter maps them with il_to_sdk",
            "parkour_kp_unique": sorted(set(kp.tolist())),
            "parkour_kd_unique": sorted(set(kd.tolist())),
            "urdf_limits_are_model_reference_not_certified_hardware_safety_limits": True,
            "urdf_effort_differs_from_contract_count": int((np.abs(urdf_effort - effort) > 1e-9).sum()),
        },
        "joints": joints,
        "interpretation_limits": [
            "Target delta divided by 20 ms is a reference-command change rate, not measured joint velocity.",
            "The captured URDF limits are comparison references, not certified controller-board safety limits.",
            "Recorded joints did not respond to inferred targets, so tracking error and closed-loop stability are not evaluated.",
            "The initial jump is relative to the first recorded standing pose; real Parkour entry conditions may differ.",
        ],
        "publishes_commands": False,
        "live_readiness": False,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--replay", type=Path, default=ROOT / "captures/policy_replay/replay.npz")
    parser.add_argument("--contract", type=Path, default=ROOT / "contract/deploy.yaml")
    parser.add_argument("--runtime-config", type=Path,
                        default=ROOT.parent / "robots/go2/config/config.yaml")
    parser.add_argument("--urdf", type=Path,
                        default=ROOT / "captures/frame_inspection_20260915/jetson_go2_description.urdf")
    parser.add_argument("--output", type=Path,
                        default=ROOT / "captures/policy_replay/target_audit.json")
    args = parser.parse_args()
    result = audit(args.replay, args.contract, args.runtime_config, args.urdf)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps({"output": str(args.output), **result["aggregate"]}, indent=2))


if __name__ == "__main__":
    main()
