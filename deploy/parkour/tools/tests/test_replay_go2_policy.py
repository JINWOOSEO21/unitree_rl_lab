import importlib.util
from pathlib import Path
import unittest

import numpy as np
import yaml


ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location("replay_go2_policy", ROOT / "tools/replay_go2_policy.py")
MOD = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MOD)


class ReplayPolicyTest(unittest.TestCase):
    def test_causal_schedule_never_selects_future(self):
        low = np.array([5, 15, 25, 35]) * 1_000_000
        scan = np.array([0, 30]) * 1_000_000
        out = MOD.build_policy_schedule(low, scan, max_low_age_ms=20, max_scan_age_ms=500)
        good = out["usable"]
        self.assertTrue(np.all(low[out["low_index"][good]] <= out["tick_steady_ns"][good]))
        self.assertTrue(np.all(scan[out["scan_index"][good]] <= out["tick_steady_ns"][good]))

    def test_history_excludes_current_and_masks_heading(self):
        h = MOD.History()
        first = np.arange(53, dtype=np.float32)
        h.prime(first)
        current = first + 100
        before = h.value().copy()
        h.push(current)
        self.assertTrue(np.array_equal(before[-53:], MOD.History.masked(first)))
        self.assertTrue(np.array_equal(h.value()[-53:], MOD.History.masked(current)))
        self.assertTrue(np.all(h.value().reshape(10, 53)[:, 6:8] == 0))

    def test_action_delay_uses_previous_raw_action(self):
        default = np.arange(12, dtype=np.float32) / 10
        delay = MOD.ActionDelay(1, (-4.8, 4.8), 0.25, default)
        a0 = np.linspace(-6, 6, 12, dtype=np.float32)
        a1 = np.full(12, 3, dtype=np.float32)
        self.assertTrue(np.allclose(delay.push(a0), default))
        self.assertTrue(np.allclose(delay.push(a1), np.clip(a0, -4.8, 4.8) * 0.25 + default))
        self.assertTrue(np.array_equal(delay.last_raw(), a1))

    def test_runtime_delay_zero_uses_current_raw_action(self):
        default = np.arange(12, dtype=np.float32) / 10
        delay = MOD.ActionDelay(0, (-4.8, 4.8), 0.25, default)
        raw = np.linspace(-6, 6, 12, dtype=np.float32)
        self.assertTrue(np.allclose(delay.push(raw), np.clip(raw, -4.8, 4.8) * 0.25 + default))

    def test_joint_and_foot_mapping(self):
        cfg = yaml.safe_load((ROOT / "contract/deploy.yaml").read_text())
        default = np.asarray(cfg["default_joint_pos"]["isaaclab"], dtype=np.float32)
        low = {
            "imu_state": {"gyroscope": [4, 8, 12], "quaternion": [1, 0, 0, 0]},
            "motor_state": [{"q": i, "dq": i * 10} for i in range(20)],
            "foot_force": [0, 3, 0, 3],
        }
        prop, contact = MOD.build_prop(
            low, default, np.asarray(cfg["index_maps"]["il_to_sdk"]),
            np.asarray(cfg["index_maps"]["il_foot_to_sdk"]), np.arange(12),
            np.array([False, False, True, False]), 2.0, 0.3, 0.0)
        self.assertTrue(np.allclose(prop[:3], [1, 2, 3]))
        self.assertTrue(np.allclose(prop[13:25] + default, cfg["index_maps"]["il_to_sdk"]))
        self.assertTrue(np.array_equal(contact, [True, False, True, False]))
        self.assertTrue(np.array_equal(prop[49:53], [0.5, -0.5, 0.5, -0.5]))

    def test_golden_observation_and_history(self):
        cfg = yaml.safe_load((ROOT / "contract/deploy.yaml").read_text())
        result = MOD.validate_golden(ROOT / "contract/golden_trace.npz", cfg)
        self.assertLessEqual(result["prop_max_abs_error"], 3e-6)
        self.assertEqual(result["history_max_abs_error"], 0)
        self.assertEqual(result["history_frames_compared"], 89)


if __name__ == "__main__":
    unittest.main()
