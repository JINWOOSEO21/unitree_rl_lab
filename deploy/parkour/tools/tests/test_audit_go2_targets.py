import importlib.util
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

import numpy as np


ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location("audit_go2_targets", ROOT / "tools/audit_go2_targets.py")
MOD = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MOD)


class TargetAuditTest(unittest.TestCase):
    def test_audit_detects_position_and_rate_violations(self):
        urdf=ROOT/'captures/frame_inspection_20260915/jetson_go2_description.urdf'
        tree=ET.parse(urdf)
        limit=tree.getroot().find("joint[@name='FL_hip_joint']/limit")
        limit.set('lower','100')
        limit.set('upper','101')
        limit.set('velocity','0')
        with tempfile.TemporaryDirectory() as temp:
            path=Path(temp)/'limits.urdf'
            tree.write(path)
            result=MOD.audit(ROOT/'captures/policy_replay/replay.npz',ROOT/'contract/deploy.yaml',
                             ROOT.parent/'robots/go2/config/config.yaml',path)
        self.assertGreater(result['aggregate']['position_limit_violations'],0)
        self.assertGreater(result['aggregate']['target_rate_samples_above_urdf_velocity'],0)

    def test_audit_rejects_nan_instead_of_reporting_no_limit_violations(self):
        with np.load(ROOT/'captures/policy_replay/replay.npz') as data:
            arrays=dict(data)
        arrays['raw_action'][0,0]=np.nan
        with tempfile.TemporaryDirectory() as temp:
            path=Path(temp)/'bad.npz'
            np.savez(path,**arrays)
            with self.assertRaisesRegex(ValueError,'non-finite'):
                MOD.audit(path,ROOT/'contract/deploy.yaml',ROOT.parent/'robots/go2/config/config.yaml',
                          ROOT/'captures/frame_inspection_20260915/jetson_go2_description.urdf')

    def test_il_to_sdk_permutation(self):
        values = np.arange(12)[None, :]
        mapping = np.array([3, 0, 9, 6, 4, 1, 10, 7, 5, 2, 11, 8])
        sdk = MOD.il_to_sdk(values, mapping)
        self.assertTrue(np.array_equal(sdk[0, mapping], values[0]))
        with self.assertRaises(ValueError):
            MOD.il_to_sdk(values, np.zeros(12, dtype=int))

    def test_pipeline_delay_zero_clips_boundaries(self):
        raw = np.array([[-9.0, -4.8, 4.8, 9.0]])
        default = np.array([0.1, 0.2, 0.3, 0.4])
        out = MOD.apply_action_pipeline(raw, 0, (-4.8, 4.8), 0.25, default)
        self.assertTrue(np.allclose(out, [[-1.1, -1.0, 1.5, 1.6]]))

    def test_pipeline_delay_one_starts_at_default(self):
        raw = np.array([[1.0, 2.0], [3.0, 4.0]])
        default = np.array([0.5, -0.5])
        out = MOD.apply_action_pipeline(raw, 1, (-4.8, 4.8), 0.25, default)
        self.assertTrue(np.allclose(out[0], default))
        self.assertTrue(np.allclose(out[1], raw[0] * 0.25 + default))

    def test_urdf_limits_require_all_named_joints(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "robot.urdf"
            path.write_text('<robot><joint name="a"><limit lower="-1" upper="1" velocity="2" effort="3"/></joint></robot>')
            self.assertEqual(MOD.load_urdf_limits(path, ["a"])["a"]["velocity"], 2.0)
            with self.assertRaises(ValueError):
                MOD.load_urdf_limits(path, ["a", "b"])


if __name__ == "__main__":
    unittest.main()
