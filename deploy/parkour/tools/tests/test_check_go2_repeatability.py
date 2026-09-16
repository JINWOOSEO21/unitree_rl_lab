from pathlib import Path
from types import SimpleNamespace
import sys
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from check_go2_repeatability import ranges,serial_add_points

class RepeatabilityTest(unittest.TestCase):
    def test_range_is_across_runs_not_joints_or_time(self):
        data=np.array([[[0,100],[5,200]],[[2,101],[8,204]]])
        result=ranges(data)
        self.assertEqual(result['max'],4)
        self.assertEqual(result['mean'],2.5)
    def test_serial_probe_preserves_shared_map_and_point_order(self):
        calls=[]
        backend=SimpleNamespace(_add_points=lambda *args,size:calls.append((args,size)))
        serial_add_points(backend)
        ids=np.array([0,0,0]);points=np.arange(9).reshape(3,3);shared=np.zeros(5)
        backend._add_points(None,None,None,None,None,None,ids,points,shared,shared,size=3)
        self.assertEqual(len(calls),3)
        for i,(args,size) in enumerate(calls):
            self.assertEqual(size,1)
            np.testing.assert_array_equal(args[7],points[i])
            self.assertIs(args[8],shared)
            self.assertIs(args[9],shared)

if __name__=='__main__':unittest.main()
