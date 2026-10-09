"""Check true interval speed and strict angle violations, without hardware."""
import importlib.util
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('retimed_audit', ROOT/'scripts/audit_retimed_trajectory.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class RetimedLimitTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.urdf = Path(self.tmp.name)/'model.urdf'
        robot = ET.Element('robot', name='test')
        self.names = ['joint_'+str(i) for i in range(56)]
        for name in self.names:
            joint = ET.SubElement(robot, 'joint', name=name, type='revolute')
            ET.SubElement(joint, 'limit', lower='-1', upper='1', velocity='1')
        ET.ElementTree(robot).write(self.urdf)
        self.data = {'time': np.array([0., .5, 1.]), 'q': np.zeros((3, 56)),
                     'joint_names': np.array(self.names), 'q_velocity': np.zeros((3, 56))}

    def test_interval_speed_ignores_misleading_velocity_array(self):
        self.data['q'][1:, 0] = 1.
        _, _, rows = module.inspect_limits(self.data, self.urdf)
        self.assertEqual(rows[0]['speed_limit_ratio'], 2.)

    def test_small_angle_violation_is_not_hidden_by_tolerance(self):
        self.data['q'][:, 0] = 1.00001
        _, _, rows = module.inspect_limits(self.data, self.urdf)
        self.assertGreater(rows[0]['position_excess_rad'], 0.)

    def test_invalid_time_names_and_nonfinite_positions_rejected(self):
        for key, value in [('time', np.array([0., 0., 1.])),
                           ('joint_names', np.array(['unknown']*56)),
                           ('q', np.full((3, 56), np.nan))]:
            bad = dict(self.data)
            bad[key] = value
            with self.subTest(key=key), self.assertRaises(ValueError):
                module.inspect_limits(bad, self.urdf)


if __name__ == '__main__':
    unittest.main()
