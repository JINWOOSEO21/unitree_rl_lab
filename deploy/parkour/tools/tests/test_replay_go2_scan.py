"""Small causal-state gates for the offline Go2 scan replay."""
from pathlib import Path
import sys
import unittest

import numpy as np

TOOLS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TOOLS))

from replay_go2_scan import _eligible_preceding, _valid_vector


class CausalPairingTest(unittest.TestCase):
    def test_future_only_state_is_missing(self):
        index, age_ms, reason = _eligible_preceding(
            np.array([200_000_000], dtype=np.int64), 100_000_000, 20.0
        )
        self.assertEqual(index, -1)
        self.assertTrue(np.isnan(age_ms))
        self.assertEqual(reason, "missing_preceding_state")

    def test_state_older_than_limit_is_rejected(self):
        index, age_ms, reason = _eligible_preceding(
            np.array([1_000_000], dtype=np.int64), 21_000_001, 20.0
        )
        self.assertEqual(index, 0)
        self.assertAlmostEqual(age_ms, 20.000001)
        self.assertEqual(reason, "state_too_old")

    def test_latest_preceding_state_is_accepted(self):
        index, age_ms, reason = _eligible_preceding(
            np.array([10_000_000, 15_000_000, 25_000_000], dtype=np.int64),
            20_000_000,
            20.0,
        )
        self.assertEqual(index, 1)
        self.assertEqual(age_ms, 5.0)
        self.assertIsNone(reason)

    def test_state_vector_requires_exact_shape_and_finite_values(self):
        self.assertTrue(_valid_vector(np.zeros(12), (12,)))
        self.assertFalse(_valid_vector(np.zeros(11), (12,)))
        value = np.zeros(12)
        value[3] = np.nan
        self.assertFalse(_valid_vector(value, (12,)))


if __name__ == "__main__":
    unittest.main()
