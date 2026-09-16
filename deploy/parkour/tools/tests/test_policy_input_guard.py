import importlib.util
from pathlib import Path
import sys
import unittest

import numpy as np


ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    "policy_input_guard", ROOT / "tools/policy_input_guard.py"
)
MOD = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MOD
SPEC.loader.exec_module(MOD)


def lowstate(quaternion=(1.0, 0.0, 0.0, 0.0)):
    return {
        "imu_state": {"gyroscope": [0.1, 0.2, 0.3], "quaternion": list(quaternion)},
        "motor_state": [{"q": 0.01 * i, "dq": -0.02 * i} for i in range(12)],
        "foot_force": [3.0, 3.0, 3.0, 3.0],
    }


class PolicyInputGuardTest(unittest.TestCase):
    def setUp(self):
        self.config = MOD.GuardConfig(
            low_receipt_timeout_ns=20,
            low_source_timeout_ns=20,
            scan_receipt_timeout_ns=500,
            scan_source_timeout_ns=500,
        )
        self.guard = MOD.PolicyInputGuard(self.config)

    def offer_pair(self, timestamp=0, source_id=1):
        self.assertTrue(self.guard.offer_low(
            lowstate(), receipt_ns=timestamp, source_ns=timestamp, source_id=source_id
        ).accepted)
        self.assertTrue(self.guard.offer_scan(
            np.zeros(132), receipt_ns=timestamp, source_ns=timestamp, source_id=source_id
        ).accepted)

    def test_valid_pair_requires_history_reset_then_acknowledges(self):
        self.offer_pair()
        decision = self.guard.evaluate(receipt_now_ns=10, source_now_ns=10)
        self.assertTrue(decision.allowed)
        self.assertTrue(decision.reset_history)
        self.guard.acknowledge_history_reset()
        self.assertFalse(self.guard.evaluate(receipt_now_ns=11, source_now_ns=11).reset_history)
        self.assertIsNot(decision.scan, self.guard._scan)

    def test_invalid_low_does_not_replace_or_refresh_last_valid_sample(self):
        self.offer_pair()
        previous = self.guard.evaluate(receipt_now_ns=5).low_provenance
        invalid = lowstate()
        invalid["motor_state"][4]["dq"] = np.nan
        result = self.guard.offer_low(invalid, receipt_ns=10, source_ns=10, source_id=2)
        self.assertFalse(result.accepted)
        decision = self.guard.evaluate(receipt_now_ns=10)
        self.assertFalse(decision.allowed)
        self.assertEqual(decision.low_provenance, previous)
        self.assertEqual(decision.low_receipt_age_ns, 10)
        self.assertTrue(any(reason.startswith("low_invalid") for reason in decision.reasons))

    def test_duplicate_scan_cannot_refresh_source_or_receipt_age(self):
        self.offer_pair()
        result = self.guard.offer_scan(
            np.ones(132), receipt_ns=400, source_ns=0, source_id=1
        )
        self.assertFalse(result.accepted)
        self.assertEqual(result.reason, "scan_duplicate_source_id")
        self.assertTrue(self.guard.offer_low(
            lowstate(), receipt_ns=501, source_ns=501, source_id=2
        ).accepted)
        decision = self.guard.evaluate(receipt_now_ns=501, source_now_ns=501)
        self.assertEqual(decision.scan_receipt_age_ns, 501)
        self.assertEqual(decision.scan_source_age_ns, 501)
        self.assertIn("scan_receipt_stale", decision.reasons)
        self.assertIn("scan_source_stale", decision.reasons)

    def test_scan_shape_nan_and_range_faults(self):
        for bad in (np.zeros(131), np.full(132, np.nan), np.full(132, 1.01)):
            guard = MOD.PolicyInputGuard(self.config)
            result = guard.offer_scan(bad, receipt_ns=0, source_ns=0, source_id=1)
            self.assertFalse(result.accepted)
            self.assertTrue(result.reason.startswith("scan_invalid"))

    def test_zero_and_nonunit_quaternion_are_rejected(self):
        for quaternion in ((0, 0, 0, 0), (2, 0, 0, 0)):
            guard = MOD.PolicyInputGuard(self.config)
            result = guard.offer_low(
                lowstate(quaternion), receipt_ns=0, source_ns=0, source_id=1
            )
            self.assertFalse(result.accepted)
            self.assertIn("quaternion norm", result.reason)

    def test_lowstate_field_shapes_are_checked(self):
        cases = []
        bad_gyro = lowstate()
        bad_gyro["imu_state"]["gyroscope"] = [0.0, 0.0]
        cases.append(bad_gyro)
        too_few_motors = lowstate()
        too_few_motors["motor_state"] = too_few_motors["motor_state"][:11]
        cases.append(too_few_motors)
        bad_feet = lowstate()
        bad_feet["foot_force"] = [1.0, 1.0, 1.0]
        cases.append(bad_feet)
        for bad in cases:
            guard = MOD.PolicyInputGuard(self.config)
            result = guard.offer_low(bad, receipt_ns=0, source_ns=0, source_id=1)
            self.assertFalse(result.accepted)
            self.assertTrue(result.reason.startswith("low_invalid"))

    def test_independent_source_age_detects_old_measurement(self):
        self.offer_pair(timestamp=100)
        decision = self.guard.evaluate(receipt_now_ns=110, source_now_ns=601)
        self.assertFalse(decision.allowed)
        self.assertNotIn("scan_receipt_stale", decision.reasons)
        self.assertIn("scan_source_stale", decision.reasons)

    def test_dropout_recovery_exposes_history_reset(self):
        self.offer_pair()
        self.assertTrue(self.guard.evaluate(receipt_now_ns=10).allowed)
        self.guard.acknowledge_history_reset()
        stale = self.guard.evaluate(receipt_now_ns=25, source_now_ns=25)
        self.assertFalse(stale.allowed)
        self.assertTrue(stale.reset_history)
        self.assertTrue(self.guard.offer_low(
            lowstate(), receipt_ns=25, source_ns=25, source_id=2
        ).accepted)
        recovered = self.guard.evaluate(receipt_now_ns=25, source_now_ns=25)
        self.assertTrue(recovered.allowed)
        self.assertTrue(recovered.reset_history)

    def test_source_regression_latches_until_explicit_stream_reset(self):
        self.offer_pair(timestamp=100, source_id=10)
        regression = self.guard.offer_scan(
            np.zeros(132), receipt_ns=110, source_ns=110, source_id=9
        )
        self.assertFalse(regression.accepted)
        self.assertTrue(regression.restart_required)
        blocked = self.guard.offer_scan(
            np.zeros(132), receipt_ns=120, source_ns=120, source_id=11
        )
        self.assertEqual(blocked.reason, "scan_restart_reset_required")

        self.guard.reset_stream("scan")
        accepted = self.guard.offer_scan(
            np.zeros(132), receipt_ns=120, source_ns=120, source_id=1
        )
        self.assertTrue(accepted.accepted)
        decision = self.guard.evaluate(receipt_now_ns=120, source_now_ns=120)
        self.assertTrue(decision.allowed)
        self.assertTrue(decision.reset_history)


if __name__ == "__main__":
    unittest.main()
