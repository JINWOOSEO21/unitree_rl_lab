import sys
import threading
from pathlib import Path
from types import SimpleNamespace as NS
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from go2_scandots_output import ScandotsOutput


class ScandotsOutputTest(unittest.TestCase):
    def setUp(self):
        self.sent=[]
        self.closed=[]
        self.output=object.__new__(ScandotsOutput)
        self.output.message_type=NS
        self.output.writer=NS(Write=lambda msg:self.sent.append(msg) is None,
                              Close=lambda:self.closed.append(True))
        self.output.lock=threading.Lock()
        self.output.active=False
        self.output.last_source=None

    def test_wire_contract_preserves_x_fast_scan_order(self):
        scan=np.linspace(-1,1,132,dtype=np.float32)
        self.output.publish(scan,[1,2,3],100_000_000,110_000_000)
        msg=self.sent[0]
        self.assertEqual((msg.width,msg.height,msg.frame_id),(12,11,'base_yaw'))
        self.assertEqual(msg.resolution,.15)
        self.assertEqual(msg.origin,[1,2])
        self.assertAlmostEqual(msg.stamp,.1)
        np.testing.assert_array_equal(msg.data,scan)

    def test_stale_future_duplicate_and_invalid_values_never_write(self):
        for scan,pos,source,now in [
            (np.zeros(131),[0,0,0],1,2),
            (np.full(132,np.nan),[0,0,0],1,2),
            (np.full(132,1.1),[0,0,0],1,2),
            (np.zeros(132),[0,np.nan,0],1,2),
            (np.zeros(132),[0,0,0],1,200_000_002),
            (np.zeros(132),[0,0,0],3,2),
        ]:
            with self.assertRaises(ValueError):self.output.publish(scan,pos,source,now)
        self.assertEqual(self.sent,[])
        self.output.publish(np.zeros(132),[0,0,0],10,20)
        for source in (10,9):
            with self.assertRaises(ValueError):self.output.publish(np.zeros(132),[0,0,0],source,20)
        self.assertEqual(len(self.sent),1)

    def test_fault_and_close_invalidate_once_then_close_writer(self):
        self.output.invalidate()
        self.assertEqual(self.sent,[])
        self.output.publish(np.zeros(132),[0,0,0],10,20)
        self.output.invalidate()
        self.assertEqual(self.sent[-1].data,[])
        self.assertFalse(self.output.active)
        self.output.invalidate()
        self.assertEqual(len(self.sent),2)
        self.output.publish(np.zeros(132),[0,0,0],30,40)
        self.output.close()
        self.assertEqual(len(self.sent),4)
        self.assertEqual(self.sent[-1].data,[])
        self.assertEqual(self.closed,[True])


if __name__=='__main__':unittest.main()
