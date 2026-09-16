"""Checks causal pairing and interpretation of offline geometric diagnostics."""
import unittest

import numpy as np

from analyze_go2_recording import horizontal_surface, preceding, read_rows, stats


class RecordingAnalysisTests(unittest.TestCase):
    def test_pairing_never_uses_future_state(self):
        times = np.array([10, 20, 30], dtype=np.int64)
        self.assertEqual(preceding(times, 9), (-1, None))
        self.assertEqual(preceding(times, 20), (1, 0.0))
        index, age = preceding(times, 25)
        self.assertEqual(index, 1)
        self.assertAlmostEqual(age, 5e-6, places=12)

    def test_known_sloped_surface(self):
        x, y = np.meshgrid(np.linspace(0.6, 1.8, 40), np.linspace(-0.8, 0.8, 35))
        z = 0.01 * x - 0.005 * y - 0.03
        result = horizontal_surface(np.column_stack([x.ravel(), y.ravel(), z.ravel()]))
        self.assertTrue(result["available"])
        np.testing.assert_allclose(result["z_equals_ax_by_c"], [0.01, -0.005, -0.03], atol=1e-12)
        self.assertLess(result["residual_rms_m"], 1e-12)

    def test_empty_surface_and_nonfinite_state(self):
        self.assertFalse(horizontal_surface(np.empty((0, 3)))["available"])
        with self.assertRaises(ValueError):
            stats([0, np.nan])

    def test_backwards_timestamps_rejected(self):
        import tempfile
        from pathlib import Path
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "state.jsonl"
            path.write_text('{"steady_ns":20}\n{"steady_ns":10}\n')
            with self.assertRaisesRegex(ValueError, "Non-monotonic"):
                read_rows(path)


if __name__ == "__main__":
    unittest.main()
