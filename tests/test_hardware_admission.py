"""Offline evidence guard regression tests; synthetic data never commands hardware."""
import csv
import importlib.util
import json
from pathlib import Path
import shutil
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location('admission', ROOT / 'scripts/check_hardware_admission.py')
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class AdmissionTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)
        shutil.copytree(ROOT / 'configs', self.root / 'configs')
        model_dir = self.root / 'outputs/robot'
        model_dir.mkdir(parents=True)
        shutil.copy(ROOT / 'outputs/robot/tron2_wuji_hand2.urdf', model_dir)

    def edit_rows(self, filename, edit):
        path = self.root / 'configs' / filename
        with path.open(newline='') as stream:
            reader = csv.DictReader(stream)
            fields, rows = reader.fieldnames, list(reader)
        for row in rows:
            edit(row)
        with path.open('w', newline='') as stream:
            writer = csv.DictWriter(stream, fields, lineterminator='\n')
            writer.writeheader()
            writer.writerows(rows)

    def test_pending_tables_block(self):
        report = MODULE.evaluate(self.root)
        self.assertFalse(report['offline_evidence_complete'])
        self.assertFalse(report['hardware_execution_authorized'])
        self.assertTrue(any('56 joints' in reason for reason in report['reasons']))

    def test_confirmed_labels_cannot_hide_unknown_or_nonfinite_limits(self):
        for value in ['unknown', 'N/A', ' ', 'nan', 'inf', '-1']:
            with self.subTest(value=value):
                self.edit_rows('hardware_limits.csv', lambda row: row.update(
                    hardware_limit_status='confirmed', hardware_speed_limit_rad_s=value))
                report = MODULE.evaluate(self.root)
                self.assertTrue(any('invalid physical numeric limits' in reason for reason in report['reasons']))
                self.assertFalse(report['hardware_execution_authorized'])

    def test_wrong_device_or_repeated_sdk_axes_block(self):
        def confirmed(row):
            row.update(mapping_status='confirmed', hardware_device_id='tron2',
                       hardware_sdk_axis='0', hardware_direction_sign='1',
                       hardware_zero_offset_rad='0', hardware_position_unit='rad')
        self.edit_rows('joint_mapping.csv', confirmed)
        report = MODULE.evaluate(self.root)
        self.assertTrue(any('invalid SDK device' in reason for reason in report['reasons']))
        self.assertTrue(any('assignments repeat' in reason for reason in report['reasons']))

    def test_bad_nested_indices_block(self):
        self.edit_rows('joint_mapping.csv', lambda row: row.update(bag_hand_index='1')
                       if row['model_joint'].startswith('l_') else None)
        report = MODULE.evaluate(self.root)
        self.assertTrue(any('bag channel indices' in reason for reason in report['reasons']))

    def test_matching_but_wrong_joint_sets_block(self):
        for filename in ['joint_mapping.csv', 'hardware_limits.csv']:
            self.edit_rows(filename, lambda row: row.update(model_joint='wrong_' + row['model_joint']))
        report = MODULE.evaluate(self.root)
        self.assertEqual(sum('exact 56' in reason for reason in report['reasons']), 2)


if __name__ == '__main__':
    unittest.main()
