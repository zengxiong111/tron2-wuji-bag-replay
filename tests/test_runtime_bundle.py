"""Exercise runtime lineage verification against synthetic, non-hardware fixtures."""
import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from argparse import Namespace

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location('verify_bundle', ROOT/'scripts/verify_bundle.py')
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)
REPLAY_SPEC = importlib.util.spec_from_file_location('replay', ROOT/'scripts/replay.py')
REPLAY = importlib.util.module_from_spec(REPLAY_SPEC)
REPLAY_SPEC.loader.exec_module(REPLAY)


class RuntimeBundleTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        for folder in ('configs', 'evidence/accepted', 'metadata', 'trajectories', 'outputs/robot', 'assets'):
            (self.root/folder).mkdir(parents=True, exist_ok=True)
        body = b'fixture bytes\n'
        digest = hashlib.sha256(body).hexdigest()
        for name in ('reference', 'execution', 'actual', 'raw_feedback', 'urdf', 'usd', 'provenance', 'retiming_manifest'):
            suffix = '.json' if name in ('provenance', 'retiming_manifest') else '.bin'
            relative = f'outputs/robot/usd{suffix}' if name == 'usd' else f'trajectories/{name}{suffix}'
            payload = b'<robot name="fixture" />\n' if name == 'urdf' else body
            (self.root/relative).write_bytes(payload)
            item_hash = hashlib.sha256(payload).hexdigest()
            setattr(self, name, {'path': relative, 'sha256': item_hash})
        provenance_path = self.root/self.provenance['path']
        provenance = {
            'reference_sha256': self.reference['sha256'],
            'execution_sha256': self.execution['sha256'],
            'raw_source_sha256': self.raw_feedback['sha256'],
            'urdf_sha256': self.urdf['sha256'],
            'usd_sha256': self.usd['sha256'],
            'retiming_manifest_sha256': self.retiming_manifest['sha256'],
        }
        provenance_path.write_text(json.dumps(provenance))
        self.provenance['sha256'] = hashlib.sha256(provenance_path.read_bytes()).hexdigest()
        model_dir = self.root/'outputs/robot'
        (model_dir/'model_changes.json').write_text(json.dumps({'usd_sha256': digest}))
        (model_dir/'asset_binding.json').write_text(json.dumps({
            'replay_usd_sha256': digest,
            'mesh_geometry_and_authored_world_transforms_equal': True,
            'physical_base_above_table_m': .45,
        }))
        profile = {'solver_position_iterations': 32, 'solver_velocity_iterations': 16,
                   'global_drive_gain_scale': 4, 'arm_drive_gain_scale': 8, 'hand_drive_gain_scale': 1,
                   'hand_velocity_limit_scale': 0.94, 'arm_force_drive': False, 'rnea_feedforward': False,
                   'target_interpolation': 'clamped C2 cubic', 'contact_offset_m': 0.001,
                   'rest_offset_m': 0.0, 'official_wuji_assembly_policy_pair_count': 20,
                   'left_thumb_flex_gain_scale': 4}
        self.binding = {'schema_version': 2, 'motion_speed': 5, 'file_time_multiplier': 1,
                        'execution_time_is_final': True, 'duration_s': 1.0, 'simulation_profile': profile,
                        **{name: getattr(self, name) for name in ('reference', 'execution', 'actual', 'raw_feedback', 'urdf', 'usd', 'provenance', 'retiming_manifest')}}
        (self.root/'configs/trajectory.json').write_text(json.dumps(self.binding))
        gates = {name: True for name in MODULE.REQUIRED_GATES}
        report = {'accepted': True, 'gates': gates, 'binding': {f'source-{name}': getattr(self, name)['sha256'] for name in ('reference', 'execution', 'actual', 'raw_feedback', 'urdf', 'usd')}}
        report_bytes = json.dumps(report, sort_keys=True).encode()
        self.report_sha = hashlib.sha256(report_bytes).hexdigest()
        (self.root/'evidence/accepted/report.json').write_bytes(report_bytes)
        (self.root/'evidence/accepted/source.json').write_bytes(report_bytes)
        physics_report = {
            'status': 'complete', 'physics_hz': 480, 'physics_steps': 33093,
            'saved_states_including_initial_and_final': 33094, 'motion_speed': 5.0,
            'replay_timing_profile': 'overall5/shared_nonuniform_time_map',
            'physical_base_to_table_m': .45, 'solver_iterations': [32, 16],
            'contact_offset_m': .001, 'rest_offset_m': 0.0,
            'hand_velocity_limit_scale': .94, 'drive_gain_scale': 4.0,
            'arm_drive_gain_scale': 8.0, 'hand_drive_gain_scale': 1.0,
            'left_thumb_flex_gain_scale': 4.0, 'runtime_self_collision': ['True'],
            'runtime_drive_settings': {f'axis-{i}': {'drive_type': 'acceleration'} for i in range(56)},
            'arm_force_drive_rnea_feedforward': {'force_drive_enabled': False, 'feedforward_enabled': False},
            'official_wuji_assembly_policy': {'enabled': True, 'official_pair_count': 20},
            'reference_sha256': self.reference['sha256'], 'execution_sha256': self.execution['sha256'],
            'raw_source_sha256': self.raw_feedback['sha256'], 'usd_sha256': self.usd['sha256'],
            'tracking_actual_sha256': self.actual['sha256'],
        }
        physics_bytes = json.dumps(physics_report, sort_keys=True).encode()
        self.physics_sha = hashlib.sha256(physics_bytes).hexdigest()
        (self.root/'evidence/accepted/physics.json').write_bytes(physics_bytes)
        acceptance = {'artifact_id': 'fixture', 'canonical_source_sha256': self.report_sha,
                      'source': {'path': 'evidence/accepted/source.json', 'sha256': self.report_sha},
                      'installed': {'path': 'evidence/accepted/source.json', 'sha256': self.report_sha},
                      'report': {'path': 'evidence/accepted/report.json', 'sha256': self.report_sha},
                      'binding_sources': {name: f'source-{name}' for name in ('reference', 'execution', 'actual', 'raw_feedback', 'urdf', 'usd')}}
        all_names = ('reference', 'execution', 'actual', 'raw_feedback', 'urdf', 'usd', 'provenance', 'retiming_manifest')
        lineage = {'runtime_files': {'configs/trajectory.json': hashlib.sha256((self.root/'configs/trajectory.json').read_bytes()).hexdigest()},
                   'imported_evidence': {'evidence/accepted/source.json': self.report_sha,
                                        'evidence/accepted/physics.json': self.physics_sha},
                   'physics_report': {'path': 'evidence/accepted/physics.json', 'sha256': self.physics_sha},
                   'acceptance': acceptance,
                   'source_to_installed_binding': {name: {'source_sha256': self.binding[name]['sha256'], 'installed_sha256': self.binding[name]['sha256'], 'transformation': 'mesh_uri_normalization_only' if name == 'urdf' else 'byte_identical'} for name in all_names},
                   'runtime_bindings': {name: self.binding[name]['sha256'] for name in all_names},
                   'source_model_semantics_sha256': MODULE.urdf_semantics_sha256(self.root, self.binding['urdf']['path']),
                   'model_identity': {'urdf_path_only_conversion': True, 'source_sha256': self.binding['urdf']['sha256'],
                                      'installed_sha256': self.binding['urdf']['sha256'],
                                      'semantic_sha256': MODULE.urdf_semantics_sha256(self.root, self.binding['urdf']['path'])}}
        (self.root/'configs/lineage.json').write_text(json.dumps(lineage))
        self.write_manifest()
        self.patch_constants()

    def write_manifest(self):
        config_hash = hashlib.sha256((self.root/'configs/trajectory.json').read_bytes()).hexdigest()
        (self.root/'release_manifest.json').write_text(json.dumps({'assets': [{'files': {'configs/trajectory.json': config_hash}}]}))

    def patch_constants(self):
        MODULE.CANONICAL_ACCEPTANCE = self.report_sha
        MODULE.ARTIFACT_ID = 'fixture'

    def test_physics_mode_forwards_isaac_python_to_wrapper(self):
        args = Namespace(physics=True, isaac_python='/opt/isaac/python.sh', headless=True,
                         out=None, max_source_seconds=2.0, max_seconds=None)
        command = REPLAY.build_command(args)
        self.assertIn(str(ROOT/'scripts/portable_fresh_physics.py'), command)
        option = command.index('--isaac-python')
        self.assertEqual(command[option+1], '/opt/isaac/python.sh')
        self.assertIn('--max-source-seconds', command)

    def test_valid_bundle_passes_and_reports_file_count(self):
        self.assertEqual(MODULE.verify(self.root), 1)

    def test_execution_double_retiming_flag_is_rejected(self):
        self.binding['file_time_multiplier'] = 5
        (self.root/'configs/trajectory.json').write_text(json.dumps(self.binding))
        self.write_manifest()
        with self.assertRaisesRegex(ValueError, '1x execution-time'):
            MODULE.verify(self.root)

    def test_changed_fresh_physics_profile_is_rejected(self):
        self.binding['simulation_profile']['arm_drive_gain_scale'] = 1
        (self.root/'configs/trajectory.json').write_text(json.dumps(self.binding))
        self.write_manifest()
        with self.assertRaisesRegex(ValueError, 'profile differs'):
            MODULE.verify(self.root)

    def test_published_robot_model_semantics_signature(self):
        if not (ROOT/'outputs/robot/tron2_wuji_hand2.urdf').is_file():
            self.skipTest('download release assets before checking the model signature')
        self.assertEqual(MODULE.urdf_semantics_sha256(ROOT, 'outputs/robot/tron2_wuji_hand2.urdf'),
                         'b2a051abb39f8591a501a38ddce017a534d0e468119c75fbd94f23ba263e3ef8')

    def test_urdf_semantics_hash_tracks_mesh_bytes_across_uri_normalization(self):
        (self.root/'assets/mesh.stl').write_bytes(b'mesh one')
        urdf = self.root/'outputs/robot/with_mesh.urdf'
        urdf.write_text('<robot name="x"><link name="l"><visual><geometry><mesh filename="package://assets/mesh.stl" /></geometry></visual></link></robot>')
        first = MODULE.urdf_semantics_sha256(self.root, 'outputs/robot/with_mesh.urdf')
        urdf.write_text(urdf.read_text().replace('package://assets/mesh.stl', 'assets/mesh.stl'))
        self.assertEqual(first, MODULE.urdf_semantics_sha256(self.root, 'outputs/robot/with_mesh.urdf'))
        (self.root/'assets/mesh.stl').write_bytes(b'mesh two')
        self.assertNotEqual(first, MODULE.urdf_semantics_sha256(self.root, 'outputs/robot/with_mesh.urdf'))

    def test_acceptance_binding_mismatch_is_rejected(self):
        gates = {name: True for name in MODULE.REQUIRED_GATES}
        report = {'accepted': True, 'gates': gates, 'binding': {f'source-{name}': getattr(self, name)['sha256'] for name in ('reference', 'execution', 'actual', 'raw_feedback', 'urdf', 'usd')}}
        report['binding']['source-reference'] = 'wrong'
        raw = json.dumps(report, sort_keys=True).encode()
        digest = hashlib.sha256(raw).hexdigest()
        (self.root/'evidence/accepted/report.json').write_bytes(raw)
        (self.root/'evidence/accepted/source.json').write_bytes(raw)
        lineage = json.loads((self.root/'configs/lineage.json').read_text())
        lineage['acceptance']['canonical_source_sha256'] = digest
        lineage['acceptance']['source']['sha256'] = digest
        lineage['acceptance']['installed']['sha256'] = digest
        lineage['acceptance']['report']['sha256'] = digest
        lineage['imported_evidence']['evidence/accepted/source.json'] = digest
        lineage['physics_report']['sha256'] = self.physics_sha
        (self.root/'configs/lineage.json').write_text(json.dumps(lineage))
        MODULE.CANONICAL_ACCEPTANCE = digest
        with self.assertRaisesRegex(ValueError, 'does not bind source reference'):
            MODULE.verify(self.root)


if __name__ == '__main__':
    unittest.main()
