"""Run a provenance-bound 5x or 6x ROS bag reference through Isaac PhysX."""
import argparse
import hashlib
import json
import time
import traceback
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUT = ROOT.parent / (ROOT.name + '-physics-runs')
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--provenance', type=Path, required=True)
parser.add_argument('--reference', type=Path, required=True, help='NPZ with time, q, joint_names')
parser.add_argument('--execution', type=Path,
                    help='Optional overall-5x execution NPZ; its samples validate reference_clock and the bound reference C2 spline')
parser.add_argument('--raw-source', type=Path,
                    help='Original raw feedback NPZ used to produce a retimed execution; required with --execution and hash-bound by provenance')
parser.add_argument('--usd', type=Path, required=True)
parser.add_argument('--out', type=Path)
parser.add_argument('--motion-speed', type=float, choices=(5, 6), required=True,
                    help='Source-motion acceleration factor; must be selected explicitly')
parser.add_argument('--wall-speed', type=float, default=1)
parser.add_argument('--physics-hz', type=int, choices=(240, 480, 960), default=240,
                    help='Physics substep frequency; simulation duration and source-clock mapping are unchanged')
parser.add_argument('--solver-position-iterations', type=int, default=32)
parser.add_argument('--solver-velocity-iterations', type=int, default=8)
parser.add_argument('--linear-targets', action='store_false', dest='smooth_targets', help='Use piecewise linear interpolation; default is clamped C2 cubic')
parser.set_defaults(smooth_targets=True)
parser.add_argument('--max-source-seconds', type=float)
parser.add_argument('--start-source-seconds', type=float, default=0)
parser.add_argument('--contact-offset', type=float, default=.001)
parser.add_argument('--rest-offset', type=float, default=.0002)
parser.add_argument('--root-sdf-resolution', type=int,
                    help='Simulation-only SDF voxel resolution override for the actual l_wrist and l_thumb_proximal_abd collision Gprims; omitted by default')
parser.add_argument('--official-wuji-assembly-policy', action='store_true',
                    help='Apply only the 20 internal body-pair exclusions authored by the pinned Wuji Beta2 with_mount MJCF; mesh geometry remains unchanged')
parser.add_argument('--velocity-limit-scale', type=float, default=1)
parser.add_argument('--hand-velocity-limit-scale', type=float, default=1,
                    help='Conservative simulation-only multiplier for the 40 hand-joint velocity caps; allowed range is 0.9 to 1.0 and acceptance remains against original URDF limits.')
parser.add_argument('--initial-trace', type=Path,
                    help='Continue from q_actual/qvel_actual in an existing trace at the selected source phase; skips cold-start settling')
parser.add_argument('--drive-gain-scale', type=float, default=1, help='Scale all Isaac simulation drive stiffness; critically damped Kd is recomputed. Does not change velocity/force limits or hardware settings.')
parser.add_argument('--arm-drive-gain-scale', type=float, default=1, help='Additional simulation-only gain multiplier for the 14 named arm joints; head and hands are unchanged.')
parser.add_argument('--hand-drive-gain-scale', type=float, default=1, help='Additional simulation-only gain multiplier for the 40 named hand joints; arms and head are unchanged.')
parser.add_argument('--arm-drive-damping-scale', type=float, default=1, help='Additional simulation-only multiplier for critically damped Kd on the 14 named arm joints; stiffness is unchanged.')
parser.add_argument('--hand-drive-damping-scale', type=float, default=1, help='Additional simulation-only multiplier for critically damped Kd on the 40 named hand joints; stiffness is unchanged.')
parser.add_argument('--left-thumb-flex-gain-scale', type=float, default=1, help='Additional simulation-only gain multiplier for l_thumb_cmc_flex; does not change velocity/force limits or hardware settings.')
parser.add_argument('--wrist-distance-feedback', action='store_true',
                    help='Enable bounded actuated wrist-gap feedback. Requires a short diagnostic interval and validated Isaac/URDF wrist Jacobians.')
parser.add_argument('--wrist-feedback-max-correction', type=float, default=.1,
                    help='Per-joint absolute position-command residual bound in radians; used only with wrist-distance feedback.')
parser.add_argument('--wrist-feedback-slew-rate', type=float, default=1.0,
                    help='Per-joint command-residual slew bound in rad/s; used only with wrist-distance feedback.')
parser.add_argument('--wrist-feedback-gain', type=float, default=1.0,
                    help='Proportional gain on predicted end-of-step wrist-gap error; used only with wrist-distance feedback.')
parser.add_argument('--wrist-feedback-integral-gain', type=float, default=5.0,
                    help='Integral gain in s^-1 on predicted wrist-gap error; bounded by the command residual trust box.')
parser.add_argument('--arm-force-drive', action='store_true',
                    help='Explicitly switch only the 14 arm drives from authored acceleration mode to force mode; default drive types are unchanged.')
parser.add_argument('--rnea-torque-position-feedforward', action='store_true',
                    help='Add bounded RNEA torque/runtime-force-stiffness position bias on the 14 arms; requires --arm-force-drive.')
parser.add_argument('--rnea-feedforward-source', type=Path,
                    help='Hash-bound C2/RNEA torque NPZ; required with --arm-force-drive-rnea-feedforward.')
parser.add_argument('--rnea-feedforward-max-correction', type=float, default=.02,
                    help='Per-arm-joint absolute position bias cap in radians for RNEA torque/Kp feedforward.')
parser.add_argument('--headless', action='store_true')
args = parser.parse_args()
if args.out is None:
    args.out = DEFAULT_OUT / f'physics_{args.motion_speed:g}x'
if min(args.wall_speed, args.motion_speed, args.contact_offset) <= 0 or not 0 <= args.rest_offset < args.contact_offset:
    raise ValueError('Invalid speed or contact offsets')
if args.solver_position_iterations < 1 or args.solver_velocity_iterations < 1:
    raise ValueError('Solver iteration counts must be positive')
if not 0 < args.velocity_limit_scale <= 1:
    raise ValueError('Velocity limit scale must preserve or tighten authored limits')
if not np.isfinite(args.hand_velocity_limit_scale) or not .9 <= args.hand_velocity_limit_scale <= 1:
    raise ValueError('Hand velocity limit scale must be finite and in [0.9, 1.0]')
if args.root_sdf_resolution is not None and args.root_sdf_resolution < 4:
    raise ValueError('Root SDF resolution must be at least 4 voxels')
if (not np.isfinite(args.drive_gain_scale) or args.drive_gain_scale <= 0
        or not np.isfinite(args.arm_drive_gain_scale) or args.arm_drive_gain_scale <= 0
        or not np.isfinite(args.hand_drive_gain_scale) or args.hand_drive_gain_scale <= 0
        or not np.isfinite(args.arm_drive_damping_scale) or args.arm_drive_damping_scale <= 0
        or not np.isfinite(args.hand_drive_damping_scale) or args.hand_drive_damping_scale <= 0
        or not np.isfinite(args.left_thumb_flex_gain_scale) or args.left_thumb_flex_gain_scale <= 0):
    raise ValueError('Drive gain scales must be finite and positive')
if (not np.isfinite(args.wrist_feedback_max_correction) or args.wrist_feedback_max_correction <= 0
        or not np.isfinite(args.wrist_feedback_slew_rate) or args.wrist_feedback_slew_rate <= 0
        or not np.isfinite(args.wrist_feedback_gain) or not 0 < args.wrist_feedback_gain <= 4
        or not np.isfinite(args.wrist_feedback_integral_gain) or args.wrist_feedback_integral_gain < 0):
    raise ValueError('Wrist feedback limits must be finite and positive, with P gain in (0, 4] and nonnegative I gain')
if args.wrist_distance_feedback and args.max_source_seconds is None:
    raise ValueError('Wrist feedback is diagnostic-only and requires an explicitly bounded --max-source-seconds interval')
if args.rnea_torque_position_feedforward and (args.rnea_feedforward_source is None or args.execution is None):
    raise ValueError('RNEA feedforward requires a bound execution and --rnea-feedforward-source')
if args.rnea_torque_position_feedforward and not args.arm_force_drive:
    raise ValueError('RNEA torque/Kp position feedforward requires the explicit --arm-force-drive option')
if args.rnea_torque_position_feedforward and not args.smooth_targets:
    raise ValueError('RNEA feedforward requires the exact smooth bound reference C2 spline; piecewise-linear target interpolation is incompatible')
if args.arm_force_drive and args.wrist_distance_feedback:
    raise ValueError('RNEA force-drive feedforward and wrist PI feedback are separate diagnostics and cannot be combined')
if not np.isfinite(args.rnea_feedforward_max_correction) or not 0 < args.rnea_feedforward_max_correction <= .02:
    raise ValueError('RNEA feedforward position-bias cap must be finite, positive, and no greater than 0.02 rad')

provenance_path = args.provenance.resolve()
ref_path, usd_path = args.reference.resolve(), args.usd.resolve()
prov = json.loads(provenance_path.read_text())
def sha(path):
    h = hashlib.sha256()
    with path.open('rb') as f:
        for block in iter(lambda: f.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()
producer_sha256_at_start = sha(Path(__file__).resolve())
ref_hash, usd_hash = sha(ref_path), sha(usd_path)
if prov.get('reference_sha256') != ref_hash or prov.get('usd_sha256') != usd_hash:
    raise ValueError('Reference/USD hashes do not match the supplied new-bag provenance')
if args.execution is not None and (args.raw_source is None or not args.official_wuji_assembly_policy):
    raise ValueError('Retimed execution requires its raw source and the explicit official Wuji assembly policy')
execution_path = args.execution.resolve() if args.execution is not None else None
raw_source_path = args.raw_source.resolve() if args.raw_source is not None else None
execution_hash = sha(execution_path) if execution_path is not None else None
raw_source_hash = sha(raw_source_path) if raw_source_path is not None else None
rnea_feedforward_path = args.rnea_feedforward_source.resolve() if args.rnea_feedforward_source is not None else None
rnea_feedforward_hash = sha(rnea_feedforward_path) if rnea_feedforward_path is not None else None
if execution_path is not None:
    if prov.get('execution_sha256') != execution_hash or prov.get('raw_source_sha256') != raw_source_hash:
        raise ValueError('Execution/raw-source hashes do not match the supplied restoration provenance')
if prov.get('bag') and prov.get('bag_sha256'):
    bag = Path(prov['bag']).resolve()
    if not bag.is_file() or sha(bag) != prov['bag_sha256']:
        raise ValueError('Bag is absent or its provenance hash changed')
data = np.load(ref_path, allow_pickle=False)
axis = np.asarray(data['time'], dtype=np.float64)
q = np.asarray(data['q'], dtype=np.float64)
names = data['joint_names'].astype(str).tolist()
original_axis = np.asarray(data['original_source_time'], dtype=np.float64) if 'original_source_time' in data.files else None
if q.shape != (len(axis), 56) or len(names) != 56 or not np.isfinite(q).all() or not np.isfinite(axis).all() or np.any(np.diff(axis) <= 0):
    raise ValueError('Expected finite, increasing, 56-DOF trajectory arrays')
if original_axis is not None:
    if original_axis.shape != axis.shape or not np.isfinite(original_axis).all() or np.any(np.diff(original_axis) <= 0):
        raise ValueError('original_source_time must be finite, strictly increasing, and aligned with reference time')
    if not np.allclose(original_axis[[0, -1]], axis[[0, -1]], rtol=0, atol=5e-8):
        raise ValueError('original_source_time must share the virtual-clock endpoints')
time_mapping_mode = 'shared_nonuniform_time_map' if original_axis is not None else 'identity_source_clock'
if args.wrist_distance_feedback and (execution_path is None or original_axis is None):
    raise ValueError('Wrist feedback requires execution-bound original_source_time and raw-source C2 targets')
if prov.get('joint_names') and list(prov['joint_names']) != names:
    raise ValueError('Joint-name order differs from provenance')
if execution_path is not None:
    execution = np.load(execution_path, allow_pickle=False)
    exe_time = np.asarray(execution['time'], dtype=np.float64)
    exe_ref_clock = np.asarray(execution['reference_clock'], dtype=np.float64)
    exe_source_time = np.asarray(execution['source_time'], dtype=np.float64)
    exe_q = np.asarray(execution['q'], dtype=np.float64)
    exe_names = execution['joint_names'].astype(str).tolist()
    exe_speed = float(execution['overall_motion_speed'])
    if (exe_names != names or exe_q.shape != (len(exe_time), 56)
            or exe_ref_clock.shape != exe_time.shape or exe_source_time.shape != exe_time.shape
            or not np.isfinite(exe_time).all() or not np.isfinite(exe_ref_clock).all()
            or not np.isfinite(exe_source_time).all() or not np.isfinite(exe_q).all()
            or np.any(np.diff(exe_time) <= 0) or np.any(np.diff(exe_ref_clock) <= 0)
            or np.any(np.diff(exe_source_time) <= 0)):
        raise ValueError('Execution NPZ must contain finite, increasing, aligned named 56-DOF axes')
    if not np.isclose(exe_speed, args.motion_speed, rtol=0, atol=1e-12):
        raise ValueError('Execution motion speed differs from the explicit replay motion speed')
    if not np.allclose(exe_ref_clock, exe_time * args.motion_speed, rtol=0, atol=1e-9):
        raise ValueError('Execution reference_clock must equal execution time times the overall motion speed')
    from scipy.interpolate import CubicSpline
    reference_spline = CubicSpline(axis, q, axis=0, bc_type=((1, np.zeros(56)), (1, np.zeros(56))))
    if exe_ref_clock[0] < axis[0] or exe_ref_clock[-1] > axis[-1]:
        raise ValueError('Execution clock extends beyond the bound reference spline')
    if not np.allclose(reference_spline(exe_ref_clock), exe_q, rtol=0, atol=1e-10):
        raise ValueError('Execution joint samples do not match the reference C2 spline at execution clocks')
    execution_derivative_validation = {'source': 'bound reference C2 spline',
                                       'evaluation_clock': 'execution.reference_clock',
                                       'qdot_time_scale': args.motion_speed,
                                       'supplied_execution_qdot_key': None,
                                       'supplied_execution_qdot_max_abs_error_rad_s': None,
                                       'validation_passed': True}
    qd_key = next((key for key in ('qd_execution', 'qdot_execution', 'qd_exec', 'qdot_exec', 'qd', 'qdot') if key in execution.files), None)
    if qd_key is not None:
        supplied_qd = np.asarray(execution[qd_key], dtype=np.float64)
        expected_qd = np.asarray(reference_spline(exe_ref_clock, 1), dtype=np.float64) * args.motion_speed
        if supplied_qd.shape != expected_qd.shape or not np.isfinite(supplied_qd).all():
            raise ValueError(f'Execution derivative array {qd_key} has invalid shape or nonfinite values: {supplied_qd.shape}')
        derivative_error = float(np.max(np.abs(supplied_qd - expected_qd)))
        if derivative_error > 1e-8:
            raise ValueError(f'Execution derivative array {qd_key} differs from the exact bound reference C2 qdot by {derivative_error:.6g} rad/s')
        execution_derivative_validation.update({'supplied_execution_qdot_key': qd_key,
                                                'supplied_execution_qdot_max_abs_error_rad_s': derivative_error})
    mapped_original = np.interp(exe_ref_clock, axis, original_axis) if original_axis is not None else exe_ref_clock
    if not np.allclose(mapped_original, exe_source_time, rtol=0, atol=5e-8):
        raise ValueError('Execution source_time does not match the reference original_source_time mapping')
    rnea_tau = None
    rnea_time_exec = None
    rnea_metadata = None
    if args.rnea_torque_position_feedforward:
        ff = np.load(rnea_feedforward_path, allow_pickle=False)
        required_ff_keys = {'time', 'reference_clock', 'torque_nm', 'joint_names', 'metadata_json'}
        if not required_ff_keys.issubset(ff.files):
            raise ValueError(f'RNEA feedforward NPZ lacks required keys: {sorted(required_ff_keys - set(ff.files))}')
        rnea_time_exec = np.asarray(ff['time'], dtype=np.float64)
        rnea_reference_clock = np.asarray(ff['reference_clock'], dtype=np.float64)
        rnea_tau = np.asarray(ff['torque_nm'], dtype=np.float64)
        ff_names = ff['joint_names'].astype(str).tolist()
        rnea_metadata = json.loads(str(ff['metadata_json'].item()))
        if (ff_names != names or rnea_tau.shape != (len(rnea_time_exec), 56)
                or rnea_reference_clock.shape != rnea_time_exec.shape
                or not np.isfinite(rnea_tau).all() or not np.isfinite(rnea_time_exec).all()
                or not np.isfinite(rnea_reference_clock).all() or np.any(np.diff(rnea_time_exec) <= 0)
                or np.any(np.diff(rnea_reference_clock) <= 0)):
            raise ValueError('RNEA torque source must contain finite, increasing, aligned 56-DOF named axes')
        if (not np.allclose(rnea_time_exec, exe_time, rtol=0, atol=1e-10)
                or not np.allclose(rnea_reference_clock, exe_ref_clock, rtol=0, atol=1e-10)):
            raise ValueError('RNEA torque source axes must exactly align with the bound execution time/reference_clock')
        urdf_path_for_ff = (ROOT / prov['urdf']).resolve()
        required_bindings = {'reference_sha256': ref_hash, 'execution_sha256': execution_hash,
                             'raw_source_sha256': raw_source_hash,
                             'urdf_sha256': sha(urdf_path_for_ff) if urdf_path_for_ff.is_file() else None}
        binding_mismatches = {key: {'expected': expected, 'found': rnea_metadata.get(key)}
                              for key, expected in required_bindings.items()
                              if expected is None or rnea_metadata.get(key) != expected}
        derivative_text = ' '.join(str(rnea_metadata.get(key, '')) for key in
                                   ('derivative_provenance', 'derivative_source', 'dynamics_method')).lower()
        expected_derivative_scales = {'motion_speed': args.motion_speed,
                                      'qdot_execution_scale': args.motion_speed,
                                      'qdd_execution_scale': args.motion_speed**2}
        derivative_scale_mismatches = {}
        for key, expected in expected_derivative_scales.items():
            try:
                found = float(rnea_metadata.get(key, np.nan))
            except (TypeError, ValueError):
                found = np.nan
            if not np.isclose(found, expected, rtol=0, atol=1e-12):
                derivative_scale_mismatches[key] = {'expected': expected, 'found': rnea_metadata.get(key)}
        if binding_mismatches or not any(token in derivative_text for token in ('c2', 'cubicspline')) or not any(token in derivative_text for token in ('rnea', 'pinocchio')):
            raise ValueError(f'RNEA source does not bind exact reference/execution/raw/URDF and C2 derivative RNEA provenance: bindings={binding_mismatches}, derivative_provenance={derivative_text!r}')
        if derivative_scale_mismatches:
            raise ValueError(f'RNEA derivative timing scales do not match the explicit overall motion speed: {derivative_scale_mismatches}')
    raw_data = np.load(raw_source_path, allow_pickle=False)
    raw_time = np.asarray(raw_data['time'], dtype=np.float64)
    raw_q = np.asarray(raw_data['q'], dtype=np.float64)
    raw_names = raw_data['joint_names'].astype(str).tolist()
    if (raw_names != names or raw_q.shape != (len(raw_time), 56) or not np.isfinite(raw_q).all()
            or not np.isfinite(raw_time).all() or np.any(np.diff(raw_time) <= 0)
            or not np.allclose(exe_source_time[[0, -1]], raw_time[[0, -1]], rtol=0, atol=5e-8)):
        raise ValueError('Raw source joint axes or source-phase endpoints differ from the execution lineage')
    from scipy.interpolate import CubicSpline
    raw_c2_spline = CubicSpline(raw_time, raw_q, axis=0, bc_type=((1, np.zeros(56)), (1, np.zeros(56))))
    # Keep the provenance-bound dense reference knots as the replay C2 source.
    # The execution samples validate/sample that spline; rebuilding a new
    # cubic from them changes qdot/qdd and would invalidate RNEA feedforward.
    manifest_ref = Path(prov.get('retiming_manifest', ''))
    manifest_path = (manifest_ref if manifest_ref.is_absolute() else ROOT/manifest_ref).resolve()
    if (not prov.get('retiming_manifest_sha256') or not manifest_path.is_file()
            or sha(manifest_path) != prov['retiming_manifest_sha256']):
        raise ValueError('Retiming manifest is absent or its hash differs from provenance')
    retime = json.loads(manifest_path.read_text())
    if (retime.get('reference_sha256') != ref_hash or retime.get('execution_sha256') != execution_hash
            or retime.get('input_sha256') != prov.get('retiming_input_sha256')):
        raise ValueError('Retiming manifest does not bind the supplied seed, reference and execution')
initial_state = None
if args.initial_trace is not None:
    initial_trace_path = args.initial_trace.resolve()
    initial_trace = np.load(initial_trace_path, allow_pickle=False)
    trace_names = initial_trace['joint_names'].astype(str).tolist()
    if trace_names != names:
        raise ValueError('Initial trace joint-name order differs from the reference')
    trace_time = np.asarray(initial_trace['source_time'], dtype=np.float64)
    trace_q = np.asarray(initial_trace['q_actual'], dtype=np.float64)
    trace_v = np.asarray(initial_trace['qvel_actual'], dtype=np.float64)
    if (trace_q.shape != (len(trace_time), 56) or trace_v.shape != trace_q.shape
            or not np.isfinite(trace_time).all() or not np.isfinite(trace_q).all()
            or not np.isfinite(trace_v).all() or np.any(np.diff(trace_time) <= 0)):
        raise ValueError('Initial trace must contain finite, increasing 56-DOF actual states and velocities')
    if args.start_source_seconds < trace_time[0] or args.start_source_seconds > trace_time[-1]:
        raise ValueError('Selected initial-state phase is not covered by the supplied trace')
    initial_state = (np.array([np.interp(args.start_source_seconds, trace_time, trace_q[:, j]) for j in range(56)]),
                     np.array([np.interp(args.start_source_seconds, trace_time, trace_v[:, j]) for j in range(56)]))
model_manifest = usd_path.parent / 'model_changes.json'
asset_binding_path = usd_path.parent / 'asset_binding.json'
if not model_manifest.is_file() or not asset_binding_path.is_file():
    raise ValueError('Collision USD requires its model_changes.json and asset_binding.json lineage')
manifest = json.loads(model_manifest.read_text())
asset_binding = json.loads(asset_binding_path.read_text())
if manifest.get('usd_sha256') != usd_hash or asset_binding.get('replay_usd_sha256') != usd_hash or not asset_binding.get('mesh_geometry_and_authored_world_transforms_equal'):
    raise ValueError('USD does not match the verified collision geometry/mount lineage')
if abs(float(asset_binding.get('physical_base_above_table_m', 0)) - .45) > 1e-8:
    raise ValueError('Collision USD binding is not verified at the fixed 0.45 m tabletop gap')
official_policy = None
if args.official_wuji_assembly_policy:
    from wuji_collision_policy import load_official_policy
    official_policy = load_official_policy()
    if official_policy.get('official_pair_count') != 20 or len(official_policy.get('body_pairs', [])) != 20:
        raise ValueError('Pinned Wuji policy must contain exactly 20 explicit body-name pairs')
    if execution_path is not None:
        if prov.get('official_policy_source_sha256') != official_policy['source_sha256']:
            raise ValueError('Official Wuji policy source hashes differ from the restoration provenance')
        if prov.get('official_policy_body_pairs') != official_policy['body_pairs']:
            raise ValueError('Official Wuji policy whitelist differs from the restoration provenance')
end = min(float(axis[-1]), args.max_source_seconds or float(axis[-1]))
if not float(axis[0]) <= args.start_source_seconds < end:
    raise ValueError('Invalid source-time interval')
if args.out.exists():
    raise FileExistsError(args.out)
if ROOT == args.out.resolve() or ROOT in args.out.resolve().parents:
    raise ValueError('Write physics records outside the repository')
args.out.mkdir(parents=True)

from isaacsim import SimulationApp
app = SimulationApp({'headless': args.headless, 'width': 1600, 'height': 1000, 'active_gpu': 0, 'multi_gpu': False})
try:
    from pxr import Gf, Usd, UsdGeom, UsdLux, UsdPhysics, PhysxSchema, PhysicsSchemaTools
    from isaacsim.core.api import World
    from isaacsim.core.prims import SingleArticulation
    from isaacsim.core.utils.stage import add_reference_to_stage
    from isaacsim.core.utils.types import ArticulationAction
    from isaacsim.core.utils.viewports import set_camera_view
    from omni.physx import get_physx_simulation_interface

    dt = 1 / args.physics_hz
    world = World(physics_dt=dt, rendering_dt=1/30, stage_units_in_meters=1)
    add_reference_to_stage(str(usd_path), '/World/Robot')
    robot = world.scene.add(SingleArticulation('/World/Robot', name='robot'))
    # Match the already-authored fixed 0.45 m layout: physical base z=0, table top z=-0.45.
    table_top_z = float(prov.get('table_top_z_m', -.45))
    if abs(table_top_z + .45) > 1e-8:
        raise ValueError(f'Fixed tabletop contract violated: table top z={table_top_z}')
    table = UsdGeom.Cube.Define(world.stage, '/World/Table')
    table.AddTranslateOp().Set(Gf.Vec3d(.8, 0, table_top_z - .025))
    table.AddScaleOp().Set(Gf.Vec3d(.6, .8, .025))
    table.CreateDisplayColorAttr([Gf.Vec3f(.10, .14, .18)])
    UsdPhysics.CollisionAPI.Apply(table.GetPrim())
    UsdLux.DomeLight.Define(world.stage, '/World/Light').CreateIntensityAttr(550)
    robot_root = world.stage.GetPrimAtPath('/World/Robot')
    runtime_filtered_pairs = []
    if official_policy is not None:
        bodies_by_name = {}
        for prim in Usd.PrimRange(robot_root):
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                bodies_by_name.setdefault(prim.GetName(), []).append(prim)
        for body_a, body_b in official_policy['body_pairs']:
            candidates_a = bodies_by_name.get(body_a, [])
            candidates_b = bodies_by_name.get(body_b, [])
            if len(candidates_a) != 1 or len(candidates_b) != 1:
                raise RuntimeError(f'Official assembly pair {body_a}/{body_b} must resolve to unique runtime rigid bodies; got {len(candidates_a)}/{len(candidates_b)}')
            prim_a, prim_b = candidates_a[0], candidates_b[0]
            filtered = UsdPhysics.FilteredPairsAPI.Apply(prim_a)
            rel = filtered.CreateFilteredPairsRel()
            existing_targets = list(rel.GetTargets())
            if prim_b.GetPath() in existing_targets:
                raise RuntimeError(f'Official runtime filtered pair already exists before opt-in: {prim_a.GetPath()} -> {prim_b.GetPath()}')
            rel.AddTarget(prim_b.GetPath())
            readback = list(filtered.GetFilteredPairsRel().GetTargets())
            if readback.count(prim_b.GetPath()) != 1:
                raise RuntimeError(f'Official runtime filtered pair did not read back exactly once: {prim_a.GetPath()} -> {prim_b.GetPath()}')
            runtime_filtered_pairs.append({'body_pair': [body_a, body_b], 'prim_a': str(prim_a.GetPath()), 'prim_b': str(prim_b.GetPath()), 'relation_targets': [str(p) for p in readback]})
        expected_name_pairs = {tuple(pair) for pair in official_policy['body_pairs']}
        if {tuple(row['body_pair']) for row in runtime_filtered_pairs} != expected_name_pairs or len(runtime_filtered_pairs) != 20:
            raise RuntimeError('Runtime official-pair application does not exactly match the 20-pair source whitelist')
    xform_cache = UsdGeom.XformCache()
    physical_base = world.stage.GetPrimAtPath('/World/Robot/' + prov.get('physical_base_prim', 'base_Link'))
    if not physical_base.IsValid():
        raise ValueError('Provenance physical_base_prim does not resolve in the referenced USD')
    base_z = float(xform_cache.GetLocalToWorldTransform(physical_base).ExtractTranslation()[2])
    actual_gap = base_z - table_top_z
    if abs(actual_gap - .45) > 1e-6:
        raise ValueError(f'Authored physical-base/table gap is {actual_gap:.9f} m, expected 0.45 m')
    # Insist contact APIs live on actual collision shapes; an API on a parent Xform is insufficient.
    shape_prims = [p for p in Usd.PrimRange(robot_root) if p.IsA(UsdGeom.Gprim) and p.HasAPI(UsdPhysics.CollisionAPI)]
    if not shape_prims:
        raise RuntimeError('No Gprim collision shapes with CollisionAPI found in runtime USD')
    shape_rest_offsets = {}
    root_sdf_shapes = {}
    for prim in shape_prims:
        PhysxSchema.PhysxCollisionAPI.Apply(prim).CreateContactOffsetAttr(args.contact_offset)
        PhysxSchema.PhysxCollisionAPI.Apply(prim).CreateRestOffsetAttr(args.rest_offset)
        shape_rest_offsets[str(prim.GetPath())] = args.rest_offset
        path = str(prim.GetPath())
        for link_name in ('l_wrist', 'l_thumb_proximal_abd'):
            if f'/{link_name}/collisions/' in path:
                attr = prim.GetAttribute('physxSDFMeshCollision:sdfResolution')
                if not attr or not attr.IsValid():
                    raise RuntimeError(f'Expected authored SDF resolution on root collision shape {path}')
                root_sdf_shapes[path] = {
                    'link': link_name,
                    'approximation': str(prim.GetAttribute('physics:approximation').Get()),
                    'authored_sdf_resolution': int(attr.Get()),
                    'runtime_sdf_resolution': int(attr.Get()),
                    'runtime_sdf_resolution_property': int(attr.Get()),
                    'effective_cooked_sdf_resolution': None,
                    'effective_resolution_note': 'PhysX may cap mesh-cooking resolution; this effective value is not exposed by the stage property and must be read from the Kit cooking log.',
                }
    for link_name in ('l_wrist', 'l_thumb_proximal_abd'):
        matches = [path for path, record in root_sdf_shapes.items() if record['link'] == link_name]
        if len(matches) != 1:
            raise RuntimeError(f'Expected exactly one actual collision Gprim for {link_name}; found {matches}')
    if args.root_sdf_resolution is not None:
        for path, record in root_sdf_shapes.items():
            prim = world.stage.GetPrimAtPath(path)
            attr = prim.GetAttribute('physxSDFMeshCollision:sdfResolution')
            attr.Set(args.root_sdf_resolution)
            record['runtime_sdf_resolution'] = int(attr.Get())
    self_collision = []
    drive_settings = {}
    velocity_caps = {}
    for prim in Usd.PrimRange(robot_root):
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            api = PhysxSchema.PhysxArticulationAPI.Apply(prim)
            api.CreateEnabledSelfCollisionsAttr(True)
            api.CreateSolverPositionIterationCountAttr(args.solver_position_iterations)
            api.CreateSolverVelocityIterationCountAttr(args.solver_velocity_iterations)
            self_collision.append(str(api.GetPrim().GetAttribute('physxArticulation:enabledSelfCollisions').Get()))
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.)
        if prim.GetName().endswith('_tip') and prim.HasAPI(UsdPhysics.MassAPI):
            mass_api = UsdPhysics.MassAPI(prim)
            if mass_api.GetMassAttr().Get() == 0:
                mass_api.GetMassAttr().Set(1e-6)
                mass_api.GetCenterOfMassAttr().Set(Gf.Vec3f(0))
                mass_api.GetDiagonalInertiaAttr().Set(Gf.Vec3f(1e-10))
        if prim.IsA(UsdPhysics.RevoluteJoint):
            attr = prim.GetAttribute('physxJoint:maxJointVelocity')
            if attr and attr.IsValid():
                authored_cap = float(attr.Get())
                hand_velocity_scale = args.hand_velocity_limit_scale if prim.GetName().startswith(('l_', 'r_')) else 1.0
                applied_velocity_scale = args.velocity_limit_scale * hand_velocity_scale
                requested_cap = authored_cap * applied_velocity_scale
                attr.Set(requested_cap)
                runtime_cap = float(attr.Get())
                if not np.isclose(runtime_cap, requested_cap, rtol=1e-6, atol=1e-6):
                    raise RuntimeError(f'Velocity-cap readback mismatch for {prim.GetName()}: requested {requested_cap}, read {runtime_cap}')
                velocity_caps[prim.GetName()] = {'authored_deg_s': authored_cap,
                                                 'global_scale': args.velocity_limit_scale,
                                                 'hand_scale': hand_velocity_scale,
                                                 'applied_scale': applied_velocity_scale,
                                                 'runtime_deg_s': runtime_cap}
            drive = UsdPhysics.DriveAPI(prim, 'angular')
            if drive:
                authored_drive_type = str(drive.GetTypeAttr().Get())
                authored_kp = float(drive.GetStiffnessAttr().Get())
                authored_kd = float(drive.GetDampingAttr().Get())
                authored_max_force = float(drive.GetMaxForceAttr().Get())
                arm_joint = prim.GetName().startswith(('proximal_', 'elbow_', 'wrist_')) and prim.GetName().endswith(('_L_Joint', '_R_Joint'))
                hand_joint = prim.GetName().startswith(('l_', 'r_'))
                if args.arm_force_drive and arm_joint:
                    drive.GetTypeAttr().Set('force')
                    if str(drive.GetTypeAttr().Get()) != 'force':
                        raise RuntimeError(f'Failed force-drive runtime readback for arm joint {prim.GetName()}')
                left_thumb_flex_joint = prim.GetName() == 'l_thumb_cmc_flex'
                joint_specific_gain_scale = args.left_thumb_flex_gain_scale if left_thumb_flex_joint else 1.0
                joint_gain_scale = (args.drive_gain_scale
                                    * (args.arm_drive_gain_scale if arm_joint else 1.0)
                                    * (args.hand_drive_gain_scale if hand_joint else 1.0)
                                    * joint_specific_gain_scale)
                requested_si_kp = authored_kp * joint_gain_scale
                critical_si_kd = float(2 * np.sqrt(max(requested_si_kp, 0)))
                damping_scale = (args.arm_drive_damping_scale if arm_joint
                                 else args.hand_drive_damping_scale if hand_joint
                                 else 1.0)
                requested_si_kd = critical_si_kd * damping_scale
                # USD rotational force drives are per degree. For the explicitly
                # scaled hand acceleration drives, also author per-degree gains
                # so the tensor's per-radian readback matches the requested SI.
                # Leave the default hand-drive path untouched for compatibility.
                hand_si_conversion = hand_joint and args.hand_drive_gain_scale != 1.0
                usd_gain_scale = (np.pi / 180.0
                                  if (args.arm_force_drive and arm_joint) or hand_si_conversion
                                  else 1.0)
                runtime_kp = requested_si_kp * usd_gain_scale
                runtime_kd = requested_si_kd * usd_gain_scale
                drive.GetStiffnessAttr().Set(runtime_kp)
                drive.GetDampingAttr().Set(runtime_kd)
                runtime_max_force = float(drive.GetMaxForceAttr().Get())
                drive_settings[prim.GetName()] = {'authored_drive_type': authored_drive_type, 'drive_type': str(drive.GetTypeAttr().Get()), 'drive_type_units': 'Nm' if str(drive.GetTypeAttr().Get()) == 'force' else 'rad/s^2', 'authored_kp': authored_kp, 'authored_kd': authored_kd, 'requested_si_kp': requested_si_kp, 'critical_si_kd': critical_si_kd, 'damping_scale': damping_scale, 'requested_si_kd': requested_si_kd, 'requested_si_gain_units': 'Nm/rad and Nm*s/rad' if str(drive.GetTypeAttr().Get()) == 'force' else 'rad/s^2 per rad and rad/s per rad', 'usd_gain_scale_from_requested_si': usd_gain_scale, 'hand_si_conversion_enabled': hand_si_conversion, 'runtime_kp': runtime_kp, 'runtime_kd': runtime_kd, 'runtime_gain_scale': joint_gain_scale, 'arm_drive_gain_scale': args.arm_drive_gain_scale if arm_joint else 1.0, 'hand_drive_gain_scale': args.hand_drive_gain_scale if hand_joint else 1.0, 'arm_drive_damping_scale': args.arm_drive_damping_scale if arm_joint else 1.0, 'hand_drive_damping_scale': args.hand_drive_damping_scale if hand_joint else 1.0, 'left_thumb_flex_gain_scale': joint_specific_gain_scale, 'authored_max_force': authored_max_force, 'max_force': runtime_max_force, 'max_force_unchanged': runtime_max_force == authored_max_force}
    missing_drives = sorted(set(names) - set(drive_settings))
    if missing_drives:
        raise RuntimeError(f'Runtime USD lacks angular drives for controlled joints: {missing_drives}')
    hand_drive_names = [name for name in names if name.startswith(('l_', 'r_'))]
    if len(hand_drive_names) != 40:
        raise RuntimeError(f'Expected exactly 40 named hand DOFs for the independent hand gain scale, found {len(hand_drive_names)}')
    hand_velocity_names = [name for name in velocity_caps if name.startswith(('l_', 'r_'))]
    if len(hand_velocity_names) != 40:
        raise RuntimeError(f'Expected exactly 40 named hand velocity caps, found {len(hand_velocity_names)}')
    if any(not np.isclose(velocity_caps[name]['hand_scale'], args.hand_velocity_limit_scale, rtol=0, atol=1e-12)
           for name in hand_velocity_names):
        raise RuntimeError('The hand velocity-limit scale was not applied to every hand axis')
    print('STAGE world.reset starting', flush=True)
    world.reset()
    print('STAGE world.reset complete', flush=True)
    if not args.headless:
        set_camera_view(np.array([2.4, .85, .35]), np.array([.1, 0, -.08]))
    ids = np.asarray([robot.get_dof_index(n) for n in names], dtype=np.int32)
    if robot.num_dof != 56 or np.any(ids < 0) or len(set(ids.tolist())) != 56:
        raise ValueError('Runtime USD joint mapping does not match the reference')
    controller = robot.get_articulation_controller()
    backend_kp, backend_kd = robot._articulation_view.get_gains()
    backend_kp = np.asarray(backend_kp, dtype=np.float64).reshape(-1)
    backend_kd = np.asarray(backend_kd, dtype=np.float64).reshape(-1)
    if backend_kp.shape != (robot.num_dof,) or backend_kd.shape != (robot.num_dof,):
        raise RuntimeError(f'Runtime backend gain readback has unexpected shape: kp={backend_kp.shape}, kd={backend_kd.shape}, dofs={robot.num_dof}')
    for i, name in enumerate(names):
        dof_i = int(robot.get_dof_index(name))
        drive_settings[name]['backend_kp'] = float(backend_kp[dof_i])
        drive_settings[name]['backend_kd'] = float(backend_kd[dof_i])
        drive_settings[name]['backend_gain_units'] = 'Nm/rad' if drive_settings[name]['drive_type'] == 'force' else 'rad/s^2 per rad'
        drive_settings[name]['usd_stiffness_units'] = 'Nm/degree' if drive_settings[name]['drive_type'] == 'force' else 'rad/s^2 per degree'
        if drive_settings[name]['drive_type'] == 'force':
            expected = drive_settings[name]['requested_si_kp']
            ratio_error = abs(float(backend_kp[dof_i]) - expected) / max(abs(expected), 1.0)
            drive_settings[name]['backend_kp_expected_from_usd_deg_conversion'] = float(expected)
            drive_settings[name]['backend_kp_conversion_relative_error'] = float(ratio_error)
            if ratio_error > 0.02:
                raise RuntimeError(f'Runtime force-drive gain unit conversion did not match USD Nm/degree to backend Nm/rad for {name}: USD={drive_settings[name]["runtime_kp"]}, backend={backend_kp[dof_i]}, expected={expected}, relative_error={ratio_error}')
        elif name.startswith(('l_', 'r_')):
            if drive_settings[name]['drive_type'] != 'acceleration':
                raise RuntimeError(f'Hand drive must remain in authored acceleration mode for simulation gain validation: {name}={drive_settings[name]["drive_type"]}')
            # With the new hand-only scale active, compensate USD's per-degree
            # angular gains to obtain the requested per-radian backend gains.
            # At the default scale, preserve the established authored behavior.
            hand_si_conversion = drive_settings[name]['hand_si_conversion_enabled']
            backend_scale = 1.0 if hand_si_conversion else 180.0 / np.pi
            expected_kp = drive_settings[name]['requested_si_kp'] * backend_scale
            expected_kd = drive_settings[name]['requested_si_kd'] * backend_scale
            kp_error = abs(float(backend_kp[dof_i]) - expected_kp) / max(abs(expected_kp), 1.0)
            kd_error = abs(float(backend_kd[dof_i]) - expected_kd) / max(abs(expected_kd), 1.0)
            drive_settings[name]['backend_kp_expected'] = float(expected_kp)
            drive_settings[name]['backend_kd_expected'] = float(expected_kd)
            drive_settings[name]['backend_kp_relative_error'] = float(kp_error)
            drive_settings[name]['backend_kd_relative_error'] = float(kd_error)
            if kp_error > 0.02 or kd_error > 0.02:
                raise RuntimeError(f'Runtime hand acceleration-drive gain readback differs from requested radian-domain gains for {name}: backend=({backend_kp[dof_i]}, {backend_kd[dof_i]}), expected=({expected_kp}, {expected_kd}), relative_error=({kp_error}, {kd_error})')
    # Record wrist separation against the raw-source C2 target in every run,
    # including controller A/B canaries with wrist feedback disabled.
    gap_backend = robot._articulation_view._physics_view
    gap_body_ids = {side: robot._articulation_view.get_body_index(f'wrist_roll_{side.upper()}_Link') for side in ('l', 'r')}
    gap_audit_enabled = raw_source_path is not None and original_axis is not None
    if gap_audit_enabled:
        import pinocchio as pin
        gap_urdf = (ROOT / prov['urdf']).resolve()
        if not gap_urdf.is_file():
            raise FileNotFoundError(f'Same-clock wrist-gap audit requires the bound composite URDF: {gap_urdf}')
        gap_model = pin.buildModelFromUrdf(str(gap_urdf))
        gap_data = gap_model.createData()
        gap_frames = {side: gap_model.getFrameId(frame) for side, frame in (('l', 'wrist_roll_L_Link'), ('r', 'wrist_roll_R_Link'))}
        if gap_model.nq != 56 or any(fid >= len(gap_model.frames) for fid in gap_frames.values()):
            raise RuntimeError('Composite URDF does not expose the expected 56-DOF wrist frames for the gap audit')
        gap_q_indices = {name: int(gap_model.joints[gap_model.getJointId(name)].idx_q) for name in names}
        if any(gap_model.getJointId(name) == 0 for name in names):
            raise RuntimeError('Composite URDF joint names do not match the replay axis for the gap audit')

    def raw_c2_wrist_gap(original_time):
        if not gap_audit_enabled:
            return float('nan')
        q_pin = np.zeros(gap_model.nq, dtype=np.float64)
        q_raw = raw_c2_spline(float(original_time))
        for col, name in enumerate(names):
            q_pin[gap_q_indices[name]] = q_raw[col]
        pin.forwardKinematics(gap_model, gap_data, q_pin)
        pin.updateFramePlacements(gap_model, gap_data)
        return float(np.linalg.norm(gap_data.oMf[gap_frames['r']].translation - gap_data.oMf[gap_frames['l']].translation))

    def actual_usd_wrist_gap():
        links = np.asarray(gap_backend.get_link_transforms(), dtype=np.float64)[0]
        return float(np.linalg.norm(links[gap_body_ids['r'], :3] - links[gap_body_ids['l'], :3]))
    first = int(np.searchsorted(axis, args.start_source_seconds))
    initial_q, initial_v = initial_state if initial_state is not None else (q[first], np.zeros(56))
    robot.set_joint_positions(initial_q, joint_indices=ids)
    robot.set_joint_velocities(initial_v, joint_indices=ids)
    feedback_state = None
    if args.wrist_distance_feedback:
        # The backend link transforms and Jacobian must correspond to the selected replay state.
        world.physics_sim_view.update_articulations_kinematic()
        import pinocchio as pin
        urdf_path = (ROOT / prov['urdf']).resolve()
        if not urdf_path.is_file():
            raise FileNotFoundError(f'Wrist feedback validation requires the bound composite URDF: {urdf_path}')
        pin_model = pin.buildModelFromUrdf(str(urdf_path))
        pin_data = pin_model.createData()
        if pin_model.nq != 56 or pin_model.nv != 56:
            raise RuntimeError(f'Expected fixed-base 56-DoF URDF for wrist feedback, got nq/nv={pin_model.nq}/{pin_model.nv}')
        wrist_frame_names = {'l': 'wrist_roll_L_Link', 'r': 'wrist_roll_R_Link'}
        frame_ids = {side: pin_model.getFrameId(wrist_frame_names[side]) for side in ('l', 'r')}
        if any(fid >= len(pin_model.frames) for fid in frame_ids.values()):
            raise RuntimeError('Composite URDF lacks named l_wrist/r_wrist FK frames')
        model_q_index = {name: int(pin_model.joints[pin_model.getJointId(name)].idx_q) for name in names}
        if any(pin_model.getJointId(name) == 0 for name in names):
            raise RuntimeError('Composite URDF joint names do not match the replay joint axis')
        arm_names = [name for name in names if name.startswith(('proximal_', 'elbow_', 'wrist_')) and name.endswith(('_L_Joint', '_R_Joint'))]
        if len(arm_names) != 14:
            raise RuntimeError(f'Expected 14 named arm joints for wrist feedback, found {len(arm_names)}')
        backend = robot._articulation_view._physics_view
        if not hasattr(backend, 'get_link_transforms'):
            raise RuntimeError('Isaac physics backend does not expose link transforms required for wrist feedback')
        body_ids = {side: robot._articulation_view.get_body_index(wrist_frame_names[side]) for side in ('l', 'r')}
        jac_probe = np.asarray(robot._articulation_view.get_jacobians())
        link_probe = np.asarray(backend.get_link_transforms())
        if jac_probe.ndim != 4 or jac_probe.shape[0] != 1 or jac_probe.shape[2:] != (6, 56):
            raise RuntimeError(f'Unexpected Isaac Jacobian shape for fixed-base articulation: {jac_probe.shape}')
        if link_probe.ndim != 3 or link_probe.shape[0] != 1 or link_probe.shape[2] != 7:
            raise RuntimeError(f'Unexpected Isaac link-transform shape: {link_probe.shape}')
        jac_rows = {side: body_ids[side] - 1 for side in ('l', 'r')}
        if any(row < 0 or row >= jac_probe.shape[1] for row in jac_rows.values()):
            raise RuntimeError(f'Isaac Jacobian link-row convention did not resolve wrists: body_ids={body_ids}, jacobian={jac_probe.shape}')

        def runtime_wrist_positions():
            links = np.asarray(backend.get_link_transforms(), dtype=np.float64)[0]
            return {side: links[body_ids[side], :3].copy() for side in ('l', 'r')}

        wrist_body_order = np.asarray([body_ids['l'], body_ids['r']], dtype=np.int32)
        from scipy.spatial.transform import Rotation

        def runtime_wrist_jacobians():
            jac_all = np.asarray(robot._articulation_view.get_jacobians(), dtype=np.float64)[0]
            links = np.asarray(backend.get_link_transforms(), dtype=np.float64)[0]
            com_local_positions, _ = robot._articulation_view.get_body_coms(body_indices=wrist_body_order)
            com_local_positions = np.asarray(com_local_positions, dtype=np.float64)[0]
            result = {}
            levers = {}
            for offset, side in enumerate(('l', 'r')):
                row = jac_rows[side]
                p_link = links[body_ids[side], :3]
                # get_body_coms returns the local COM pose. Rotate its local offset by the runtime link pose.
                com_local = com_local_positions[offset]
                link_rotation = Rotation.from_quat(links[body_ids[side], 3:7])
                r_link_from_com = -link_rotation.apply(com_local)
                j_com_linear = jac_all[row, :3, :]
                j_angular = jac_all[row, 3:6, :]
                # PhysX reports the linear rows at the runtime COM. Shift each spatial column to the link origin.
                shifted = j_com_linear + np.cross(j_angular.T, r_link_from_com[None, :]).T
                result[side] = {'raw': j_com_linear, 'com_shifted_to_link_origin': shifted}
                levers[side] = r_link_from_com.copy()
            return result, levers

        def pin_wrist_positions(q_values):
            q_pin = np.zeros(pin_model.nq, dtype=np.float64)
            for col, name in enumerate(names):
                q_pin[model_q_index[name]] = q_values[col]
            pin.forwardKinematics(pin_model, pin_data, q_pin)
            pin.updateFramePlacements(pin_model, pin_data)
            return {side: pin_data.oMf[frame_ids[side]].translation.copy() for side in ('l', 'r')}

        q_check = np.asarray(robot.get_joint_positions(joint_indices=ids), dtype=np.float64)
        usd_pos = runtime_wrist_positions()
        pin_pos = pin_wrist_positions(q_check)
        fk_position_error = {side: float(np.linalg.norm(usd_pos[side] - pin_pos[side])) for side in ('l', 'r')}
        if max(fk_position_error.values()) > 5e-5:
            raise RuntimeError(f'Runtime USD wrist_roll link positions disagree with composite URDF FK by >0.05 mm: {fk_position_error}; check base/world transform and selected link frame')
        jac_candidates, com_levers = runtime_wrist_jacobians()
        jac_now = np.asarray(robot._articulation_view.get_jacobians(), dtype=np.float64)[0]
        fd_epsilon = 1e-3
        jacobian_fd_errors = {'runtime_usd': {}, 'urdf_pinocchio': {}}
        jacobian_convention_probe = {}
        for side in ('l', 'r'):
            for name in arm_names:
                col = int(robot.get_dof_index(name))
                q_perturbed = q_check.copy()
                q_perturbed[names.index(name)] += fd_epsilon
                robot.set_joint_positions(q_perturbed, joint_indices=ids)
                world.physics_sim_view.update_articulations_kinematic()
                runtime_fd = (runtime_wrist_positions()[side] - usd_pos[side]) / fd_epsilon
                pin_fd = (pin_wrist_positions(q_perturbed)[side] - pin_pos[side]) / fd_epsilon
                isaac_raw_col = jac_now[jac_rows[side], :3, col]
                isaac_shifted_col = jac_candidates[side]['com_shifted_to_link_origin'][:, col]
                raw_runtime_err = float(np.linalg.norm(runtime_fd - isaac_raw_col))
                shifted_runtime_err = float(np.linalg.norm(runtime_fd - isaac_shifted_col))
                jacobian_fd_errors['runtime_usd'][f'{side}:{name}'] = {'raw': raw_runtime_err, 'com_shifted_to_link_origin': shifted_runtime_err}
                jacobian_fd_errors['urdf_pinocchio'][f'{side}:{name}'] = {
                    'raw': float(np.linalg.norm(pin_fd - isaac_raw_col)),
                    'com_shifted_to_link_origin': float(np.linalg.norm(pin_fd - isaac_shifted_col)),
                    'runtime_usd_vs_urdf_fd': float(np.linalg.norm(runtime_fd - pin_fd)),
                }
                alternatives = {}
                for row in range(max(0, body_ids[side] - 2), min(jac_now.shape[0], body_ids[side] + 1)):
                    for block_name, block in (('first3', slice(0, 3)), ('last3', slice(3, 6))):
                        alternatives[f'row{row}_{block_name}'] = float(np.linalg.norm(runtime_fd - jac_now[row, block, col]))
                alternatives['runtime_usd_fd_vector_m_per_rad'] = runtime_fd.tolist()
                alternatives['urdf_fd_vector_m_per_rad'] = pin_fd.tolist()
                alternatives['raw_jacobian_vector_m_per_rad'] = isaac_raw_col.tolist()
                alternatives['com_shifted_jacobian_vector_m_per_rad'] = isaac_shifted_col.tolist()
                alternatives['com_local_m'] = ( -Rotation.from_quat(np.asarray(backend.get_link_transforms(), dtype=np.float64)[0, body_ids[side], 3:7]).inv().apply(com_levers[side])).tolist()
                alternatives['com_to_link_lever_world_m'] = com_levers[side].tolist()
                alternatives['link_origin_world_m'] = usd_pos[side].tolist()
                jacobian_convention_probe[f'{side}:{name}'] = alternatives
        robot.set_joint_positions(q_check, joint_indices=ids)
        robot.set_joint_velocities(initial_v, joint_indices=ids)
        world.physics_sim_view.update_articulations_kinematic()
        raw_max_error = max(v['raw'] for v in jacobian_fd_errors['runtime_usd'].values())
        shifted_max_error = max(v['com_shifted_to_link_origin'] for v in jacobian_fd_errors['runtime_usd'].values())
        jacobian_reference_point = 'raw_runtime_jacobian' if raw_max_error <= shifted_max_error else 'com_shifted_to_link_origin'
        selected_max_error = min(raw_max_error, shifted_max_error)
        if selected_max_error > .001:
            worst = max(jacobian_fd_errors['runtime_usd'], key=lambda k: min(jacobian_fd_errors['runtime_usd'][k].values()))
            raise RuntimeError(f'Isaac wrist_roll Jacobian does not match runtime USD finite differences by <=0.001 m/rad: raw_max={raw_max_error:.6g}, COM_shifted_max={shifted_max_error:.6g}, selected={jacobian_reference_point}, worst_candidate={worst}: {jacobian_convention_probe[worst]}; body_indices={body_ids}; articulation_bodies={robot._articulation_view.body_names}')
        feedback_state = {
            'pin_model': pin_model, 'pin_data': pin_data, 'frame_ids': frame_ids,
            'model_q_index': model_q_index, 'body_ids': body_ids, 'jac_rows': jac_rows,
            'arm_names': arm_names, 'backend': backend,
            'runtime_wrist_positions': runtime_wrist_positions,
            'runtime_wrist_jacobians': runtime_wrist_jacobians,
            'jacobian_reference_point': jacobian_reference_point,
            'pin_wrist_positions': pin_wrist_positions,
            'dof_limits': np.asarray(robot._articulation_view.get_dof_limits(), dtype=np.float64)[0],
            'validation': {'urdf': str(urdf_path.resolve()), 'runtime_usd_wrist_vs_urdf_fk_error_m': fk_position_error,
                           'finite_difference_epsilon_rad': fd_epsilon,
                           'jacobian_fd_error_m_per_rad': jacobian_fd_errors,
                           'jacobian_reference_point': jacobian_reference_point,
                           'jacobian_max_runtime_fd_error_m_per_rad': selected_max_error,
                           'jacobian_convention_probe_m_per_rad': jacobian_convention_probe,
                           'com_to_link_lever_world_m': {side: vector.tolist() for side, vector in com_levers.items()},
                           'jacobian_shape': list(jac_probe.shape), 'link_transform_shape': list(link_probe.shape),
                           'wrist_links': wrist_frame_names, 'body_indices': body_ids, 'jacobian_rows': jac_rows,
                           'validation_passed': True},
            'previous_residual': np.zeros(56, dtype=np.float64),
            'integral_residual': np.zeros(56, dtype=np.float64),
        }
    from scipy.interpolate import CubicSpline
    if args.smooth_targets:
        spline = reference_spline if execution_path is not None else CubicSpline(axis, q, axis=0, bc_type=((1, np.zeros(56)), (1, np.zeros(56))))
    if initial_state is None:
        initial_target_v = np.zeros(56)
    elif args.smooth_targets:
        initial_target_v = np.asarray(spline(args.start_source_seconds, 1)) * args.motion_speed
    else:
        initial_target_v = ((q[first+1] - q[first]) / (axis[first+1] - axis[first])) * args.motion_speed
    controller.apply_action(ArticulationAction(joint_positions=q[first], joint_velocities=initial_target_v, joint_indices=ids))
    if initial_state is None:
        settling_steps = int(round(.5 / dt))
        print(f'STAGE settling {settling_steps} steps ({settling_steps * dt:.6f}s) starting', flush=True)
        for _ in range(settling_steps):
            world.step(render=False)
        print('STAGE settling complete', flush=True)
    else:
        print('STAGE initial actual state restored; cold-start settling skipped', flush=True)

    frame_contacts = []
    def contact_callback(headers, contacts):
        for header in headers:
            if not header.num_contact_data:
                continue
            a = str(PhysicsSchemaTools.intToSdfPath(header.actor0))
            b = str(PhysicsSchemaTools.intToSdfPath(header.actor1))
            seps = [float(c.separation) for c in contacts[header.contact_data_offset:header.contact_data_offset+header.num_contact_data]]
            impulses = [float(np.linalg.norm(c.impulse)) for c in contacts[header.contact_data_offset:header.contact_data_offset+header.num_contact_data]]
            frame_contacts.append({'body0': a, 'body1': b, 'contact_count': len(seps), 'min_separation_m': min(seps), 'max_impulse_norm': max(impulses)})
    subscription = get_physx_simulation_interface().subscribe_contact_report_events(contact_callback)
    src_t, original_src_t, sim_axis, target_q, command_q, command_residual_q, actual_q, actual_v, applied_effort_q, measured_effort_q, actual_wrist_gap, desired_wrist_gap, contact_rows = [], [], [], [], [], [], [], [], [], [], [], [], []
    rnea_feedforward_tau_q, rnea_feedforward_residual_q = [], []
    rnea_feedforward_q_limit_clip_count = 0
    world.physics_sim_view.update_articulations_kinematic()
    initial_actual = np.asarray(robot.get_joint_positions(joint_indices=ids), dtype=np.float64)
    initial_v = np.asarray(robot.get_joint_velocities(joint_indices=ids), dtype=np.float64)
    initial_wrist_gap = actual_usd_wrist_gap()
    initial_original_time = float(np.interp(args.start_source_seconds, axis, original_axis)) if original_axis is not None else args.start_source_seconds
    initial_desired_gap = raw_c2_wrist_gap(initial_original_time)
    src_t.append(args.start_source_seconds)
    if original_axis is not None:
        original_src_t.append(float(np.interp(args.start_source_seconds, axis, original_axis)))
    sim_axis.append(0.); target_q.append(q[first].copy()); command_q.append(q[first].copy()); command_residual_q.append(np.zeros(56)); actual_q.append(initial_actual); actual_v.append(initial_v); applied_effort_q.append(np.full(56, np.nan)); measured_effort_q.append(np.full(56, np.nan)); contact_rows.append([])
    rnea_feedforward_tau_q.append(np.full(56, np.nan)); rnea_feedforward_residual_q.append(np.zeros(56))
    actual_wrist_gap.append(initial_wrist_gap); desired_wrist_gap.append(initial_desired_gap)
    elapsed_source = end - args.start_source_seconds
    requested_sim_duration = elapsed_source / args.motion_speed
    full_step_count = int(np.floor(requested_sim_duration / dt + 1e-10))
    final_step_dt = requested_sim_duration - full_step_count * dt
    if final_step_dt < 1e-10:
        final_step_dt = 0.0
    step_count = full_step_count + int(final_step_dt > 0)
    start = time.perf_counter(); next_render = 0.
    print(f'STAGE replay loop starting steps={step_count}', flush=True)
    for step in range(1, step_count + 1):
        step_dt = final_step_dt if final_step_dt > 0 and step == step_count else dt
        if step_dt != dt:
            world.set_simulation_dt(physics_dt=step_dt, rendering_dt=1/30)
        sim_t = sim_axis[-1] + step_dt
        t = min(args.start_source_seconds + sim_t * args.motion_speed, end)
        j = min(int(np.searchsorted(axis, t, side='right') - 1), len(axis) - 2)
        alpha = float(np.clip((t - axis[j]) / (axis[j+1] - axis[j]), 0, 1))
        target = q[j] * (1-alpha) + q[j+1] * alpha
        target_v = (q[j+1] - q[j]) / (axis[j+1]-axis[j]) * args.motion_speed
        if args.smooth_targets:
            target = np.asarray(spline(t))
            target_v = np.asarray(spline(t, 1)) * args.motion_speed
        command_residual = np.zeros(56, dtype=np.float64)
        feedforward_tau = np.zeros(56, dtype=np.float64)
        feedforward_residual = np.zeros(56, dtype=np.float64)
        desired_gap = float('nan')
        original_phase = float(np.interp(t, axis, original_axis)) if original_axis is not None else t
        if gap_audit_enabled:
            desired_gap = raw_c2_wrist_gap(original_phase)
        if feedback_state is not None:
            current_wrist = feedback_state['runtime_wrist_positions']()
            raw_c2_target = np.asarray(raw_c2_spline(original_phase), dtype=np.float64)
            desired_wrist = feedback_state['pin_wrist_positions'](raw_c2_target)
            delta = current_wrist['r'] - current_wrist['l']
            gap = float(np.linalg.norm(delta))
            if not np.isfinite(gap) or gap < 1e-6:
                raise RuntimeError(f'Invalid measured wrist distance at physics step {step}: {gap}')
            direction = delta / gap
            wrist_jac, _ = feedback_state['runtime_wrist_jacobians']()
            jac_l = wrist_jac['l'][feedback_state['jacobian_reference_point']]
            jac_r = wrist_jac['r'][feedback_state['jacobian_reference_point']]
            distance_jac = direction @ (jac_r - jac_l)
            arm_cols = np.asarray([int(robot.get_dof_index(name)) for name in feedback_state['arm_names']], dtype=np.int32)
            active_jac = distance_jac[arm_cols]
            denom = float(active_jac @ active_jac)
            if not np.isfinite(denom) or denom < 1e-8:
                raise RuntimeError(f'Wrist distance has inadequate arm Jacobian authority at physics step {step}: {denom}')
            desired_gap = float(np.linalg.norm(desired_wrist['r'] - desired_wrist['l']))
            velocity_backend = np.asarray(robot.get_joint_velocities(), dtype=np.float64).reshape(-1)
            if velocity_backend.shape != (56,) or not np.isfinite(velocity_backend).all():
                raise RuntimeError(f'Invalid backend joint velocity vector for wrist prediction: {velocity_backend.shape}')
            predicted_gap = gap + float(distance_jac @ velocity_backend) * step_dt
            gap_error = desired_gap - predicted_gap
            jacobian_right_inverse = active_jac / denom
            prior = feedback_state['previous_residual'][np.asarray([names.index(n) for n in feedback_state['arm_names']])]
            max_step = args.wrist_feedback_slew_rate * step_dt
            integral_prior = feedback_state['integral_residual'][np.asarray([names.index(n) for n in feedback_state['arm_names']])]
            integral_delta = args.wrist_feedback_integral_gain * gap_error * step_dt * jacobian_right_inverse
            unsaturated_total = args.wrist_feedback_gain * gap_error * jacobian_right_inverse + integral_prior + integral_delta
            # Conditional integration prevents windup against the absolute trust box.
            outward = (np.abs(unsaturated_total) > args.wrist_feedback_max_correction) & (np.sign(integral_delta) == np.sign(unsaturated_total))
            integral_next = np.where(outward, integral_prior, integral_prior + integral_delta)
            integral_next = np.clip(integral_next, -args.wrist_feedback_max_correction, args.wrist_feedback_max_correction)
            requested = args.wrist_feedback_gain * gap_error * jacobian_right_inverse + integral_next
            # A per-joint trust box and slew bound keep the command residual smooth and reversible.
            requested = np.clip(requested, -args.wrist_feedback_max_correction, args.wrist_feedback_max_correction)
            bounded = prior + np.clip(requested - prior, -max_step, max_step)
            for arm_i, name in enumerate(feedback_state['arm_names']):
                target_col = names.index(name)
                dof_col = int(robot.get_dof_index(name))
                lower, upper = feedback_state['dof_limits'][dof_col]
                command_residual[target_col] = np.clip(bounded[arm_i], lower - target[target_col], upper - target[target_col])
                feedback_state['integral_residual'][target_col] = integral_next[arm_i]
            feedback_state['previous_residual'] = command_residual.copy()
        if args.rnea_torque_position_feedforward:
            arm_names_for_ff = [name for name in names if name.startswith(('proximal_', 'elbow_', 'wrist_')) and name.endswith(('_L_Joint', '_R_Joint'))]
            if len(arm_names_for_ff) != 14:
                raise RuntimeError(f'Expected 14 named arm joints for RNEA feedforward, found {len(arm_names_for_ff)}')
            execution_time = float(np.interp(t, exe_ref_clock, exe_time))
            ff_right = min(int(np.searchsorted(rnea_time_exec, execution_time, side='right')), len(rnea_time_exec)-1)
            ff_left = max(0, ff_right - 1)
            ff_alpha = 0.0 if ff_left == ff_right else float((execution_time-rnea_time_exec[ff_left])/(rnea_time_exec[ff_right]-rnea_time_exec[ff_left]))
            tau_sample = rnea_tau[ff_left] * (1 - ff_alpha) + rnea_tau[ff_right] * ff_alpha
            dof_limits_now = np.asarray(robot._articulation_view.get_dof_limits(), dtype=np.float64)[0]
            for name in arm_names_for_ff:
                joint_col = names.index(name)
                kp_force = float(drive_settings[name]['backend_kp'])
                force_cap = float(drive_settings[name]['max_force'])
                if drive_settings[name]['drive_type'] != 'force' or kp_force <= 0:
                    raise RuntimeError(f'RNEA position bias requires a runtime force drive with positive stiffness: {name}, settings={drive_settings[name]}')
                if not drive_settings[name]['max_force_unchanged']:
                    raise RuntimeError(f'RNEA position bias requires the authored actuator max-force cap to remain unchanged: {name}, settings={drive_settings[name]}')
                torque = float(tau_sample[joint_col])
                if abs(torque) > force_cap + 1e-5:
                    raise RuntimeError(f'Bound RNEA torque exceeds unchanged authored actuator force cap for {name}: {torque:.6g} Nm > {force_cap:.6g} Nm')
                bias = float(np.clip(torque / kp_force, -args.rnea_feedforward_max_correction, args.rnea_feedforward_max_correction))
                dof_col = int(robot.get_dof_index(name))
                lower, upper = dof_limits_now[dof_col]
                bounded_bias = float(np.clip(bias, lower-target[joint_col], upper-target[joint_col]))
                rnea_feedforward_q_limit_clip_count += int(abs(bounded_bias-bias) > 1e-12)
                feedforward_tau[joint_col] = torque
                feedforward_residual[joint_col] = bounded_bias
            command_residual += feedforward_residual
        q_command = target + command_residual
        controller.apply_action(ArticulationAction(joint_positions=q_command, joint_velocities=target_v, joint_indices=ids))
        world.step(render=False)
        actual = np.asarray(robot.get_joint_positions(joint_indices=ids), dtype=np.float64)
        velocity = np.asarray(robot.get_joint_velocities(joint_indices=ids), dtype=np.float64)
        try:
            applied_effort = np.asarray(robot.get_applied_joint_efforts(joint_indices=ids), dtype=np.float64)
        except (AttributeError, TypeError):
            applied_effort = np.full(56, np.nan)
        try:
            measured_effort = np.asarray(robot.get_measured_joint_efforts(joint_indices=ids), dtype=np.float64)
        except (AttributeError, TypeError):
            measured_effort = np.full(56, np.nan)
        if not np.isfinite(actual).all() or not np.isfinite(velocity).all():
            raise RuntimeError(f'Non-finite runtime state at physics step {step}')
        post_gap = actual_usd_wrist_gap()
        src_t.append(t)
        if original_axis is not None:
            original_src_t.append(float(np.interp(t, axis, original_axis)))
        sim_axis.append(sim_t); target_q.append(target.copy()); command_q.append(q_command.copy()); command_residual_q.append(command_residual.copy()); actual_q.append(actual.copy()); actual_v.append(velocity.copy()); applied_effort_q.append(applied_effort.copy()); measured_effort_q.append(measured_effort.copy()); rnea_feedforward_tau_q.append(feedforward_tau.copy()); rnea_feedforward_residual_q.append(feedforward_residual.copy())
        actual_wrist_gap.append(post_gap); desired_wrist_gap.append(desired_gap)
        contact_rows.append(frame_contacts); frame_contacts = []
        if step % (args.physics_hz * 5) == 0 or step == step_count:
            checkpoint = args.out/'checkpoint_actual.npz'
            temp_checkpoint = args.out/'checkpoint_actual.tmp.npz'
            trace_payload = {'source_time': np.asarray(src_t), 'simulation_time': np.asarray(sim_axis), 'q_target': np.asarray(target_q), 'q_command': np.asarray(command_q), 'q_command_residual': np.asarray(command_residual_q), 'rnea_feedforward_tau_nm': np.asarray(rnea_feedforward_tau_q), 'rnea_feedforward_residual_rad': np.asarray(rnea_feedforward_residual_q), 'q_actual': np.asarray(actual_q), 'qvel_actual': np.asarray(actual_v), 'wrist_gap_actual_m': np.asarray(actual_wrist_gap), 'wrist_gap_raw_c2_desired_m': np.asarray(desired_wrist_gap), 'applied_joint_efforts': np.asarray(applied_effort_q), 'measured_joint_efforts': np.asarray(measured_effort_q), 'joint_names': np.asarray(names), 'frame_rate_hz': np.asarray([args.physics_hz]), 'physics_hz': np.asarray([args.physics_hz]), 'motion_speed': np.asarray([args.motion_speed]), 'time_mapping_mode': np.asarray([time_mapping_mode])}
            if original_axis is not None:
                trace_payload['original_source_time'] = np.asarray(original_src_t)
            np.savez_compressed(temp_checkpoint, **trace_payload)
            temp_checkpoint.replace(checkpoint)
            print('PHYSICS_CHECKPOINT', step, '/', step_count, 'source', round(t, 3), flush=True)
        if step % args.physics_hz == 0:
            print('PHYSICS_PROGRESS', step, '/', step_count, 'source', round(t, 3), 'max_error', round(float(np.max(np.abs(actual-target))), 5), flush=True)
        if sim_t >= next_render:
            world.render(); next_render = sim_t + args.wall_speed / 30
        delay = sim_t / args.wall_speed + start - time.perf_counter()
        if delay > 0:
            time.sleep(delay)
    trace_payload = {'source_time': np.asarray(src_t), 'simulation_time': np.asarray(sim_axis), 'q_target': np.asarray(target_q), 'q_command': np.asarray(command_q), 'q_command_residual': np.asarray(command_residual_q), 'rnea_feedforward_tau_nm': np.asarray(rnea_feedforward_tau_q), 'rnea_feedforward_residual_rad': np.asarray(rnea_feedforward_residual_q), 'q_actual': np.asarray(actual_q), 'qvel_actual': np.asarray(actual_v), 'wrist_gap_actual_m': np.asarray(actual_wrist_gap), 'wrist_gap_raw_c2_desired_m': np.asarray(desired_wrist_gap), 'applied_joint_efforts': np.asarray(applied_effort_q), 'measured_joint_efforts': np.asarray(measured_effort_q), 'joint_names': np.asarray(names), 'frame_rate_hz': np.asarray([args.physics_hz]), 'physics_hz': np.asarray([args.physics_hz]), 'motion_speed': np.asarray([args.motion_speed]), 'time_mapping_mode': np.asarray([time_mapping_mode])}
    if original_axis is not None:
        trace_payload['original_source_time'] = np.asarray(original_src_t)
    np.savez_compressed(args.out/'tracking_actual.npz', **trace_payload)
    (args.out/'contacts_per_physics_step.json').write_text(json.dumps(contact_rows, separators=(',', ':')))
    tracking_hash = sha(args.out/'tracking_actual.npz')
    qactual_np, qvel_np = np.asarray(actual_q), np.asarray(actual_v)
    qtarget_np = np.asarray(target_q)
    effort_np = np.asarray(applied_effort_q)
    measured_effort_np = np.asarray(measured_effort_q)
    qfd_np = np.diff(qactual_np, axis=0) / np.diff(np.asarray(sim_axis))[:, None]
    target_error = qactual_np-qtarget_np
    physx_peak = np.max(np.abs(qvel_np), axis=0)
    physx_rms = np.sqrt(np.mean(qvel_np**2, axis=0))
    qfd_peak = np.max(np.abs(qfd_np), axis=0)
    qfd_rms = np.sqrt(np.mean(qfd_np**2, axis=0))
    error_peak = np.max(np.abs(target_error), axis=0)
    error_rms = np.sqrt(np.mean(target_error**2, axis=0))
    caps_rad = {name: float(np.deg2rad(values['runtime_deg_s'])) for name, values in velocity_caps.items()}
    original_caps_rad = {name: float(np.deg2rad(values['authored_deg_s'])) for name, values in velocity_caps.items()}
    contact_events = [event for row in contact_rows for event in row]
    min_separation = min((event['min_separation_m'] for event in contact_events), default=None)
    gap_actual_np = np.asarray(actual_wrist_gap, dtype=np.float64)
    gap_desired_np = np.asarray(desired_wrist_gap, dtype=np.float64)
    gap_error_mm = (gap_actual_np - gap_desired_np) * 1000.0
    finite_gap_error_mm = gap_error_mm[np.isfinite(gap_error_mm)]
    trace_intervals = np.diff(np.asarray(sim_axis, dtype=np.float64))
    report = {'status': 'complete', 'physics_hz': args.physics_hz, 'physics_steps': step_count, 'saved_states_including_initial_and_final': len(src_t), 'contact_rows_per_physics_step': len(contact_rows), 'contact_rows_match_saved_states': len(contact_rows) == len(src_t), 'trace_interval_s_min': float(np.min(trace_intervals)), 'trace_interval_s_max': float(np.max(trace_intervals)), 'tracking_actual_sha256': tracking_hash, 'contacts_sha256': sha(args.out/'contacts_per_physics_step.json'), 'motion_speed': args.motion_speed, 'motion_speed_semantics': 'overall speed with shared local retiming map' if original_axis is not None else 'uniform source-clock motion speed', 'replay_timing_profile': f'overall{args.motion_speed:g}/shared_nonuniform_time_map' if original_axis is not None else f'uniform_{args.motion_speed:g}x_source_clock', 'time_mapping_mode': time_mapping_mode, 'wall_speed_requested': args.wall_speed, 'target_interpolation': 'bound reference C2 cubic' if execution_path is not None and args.smooth_targets else ('clamped C2 cubic' if args.smooth_targets else 'piecewise linear'), 'execution_derivative_validation': execution_derivative_validation if execution_path is not None else None, 'position_target_spline_source': 'same dense provenance-bound reference C2 spline used for q and qdot' if execution_path is not None and args.smooth_targets else None, 'source_interval_s': [args.start_source_seconds, end], 'original_source_interval_s': [original_src_t[0], original_src_t[-1]] if original_axis is not None else None, 'requested_source_duration_s': elapsed_source, 'simulation_duration_s': float(sim_axis[-1]), 'wall_elapsed_s': time.perf_counter()-start, 'final_source_time_s': src_t[-1], 'final_original_source_time_s': original_src_t[-1] if original_axis is not None else None, 'bag': prov.get('bag'), 'bag_sha256': prov.get('bag_sha256'), 'provenance_sha256': sha(provenance_path), 'reference': str(ref_path), 'reference_sha256': ref_hash, 'execution': str(execution_path) if execution_path else None, 'execution_sha256': execution_hash, 'raw_source': str(raw_source_path) if raw_source_path else None, 'raw_source_sha256': raw_source_hash, 'raw_c2_wrist_gap_audit_enabled': gap_audit_enabled, 'raw_c2_wrist_gap_actual_vs_desired_peak_error_mm': float(np.max(np.abs(finite_gap_error_mm))) if finite_gap_error_mm.size else None, 'raw_c2_wrist_gap_actual_vs_desired_rms_error_mm': float(np.sqrt(np.mean(finite_gap_error_mm**2))) if finite_gap_error_mm.size else None, 'raw_c2_wrist_gap_trace_semantics': 'actual PhysX wrist_roll link-origin gap vs Pinocchio raw-source clamped C2 gap evaluated at mapped original_source_time; reference clock remains virtual' if gap_audit_enabled else None, 'usd': str(usd_path), 'usd_sha256': usd_hash, 'producer_sha256': producer_sha256_at_start, 'producer_sha256_at_shutdown': sha(Path(__file__).resolve()), 'producer_source_changed_during_run': sha(Path(__file__).resolve()) != producer_sha256_at_start, 'physical_base_prim': str(physical_base.GetPath()), 'physical_base_z_m': base_z, 'table_top_z_m': table_top_z, 'physical_base_to_table_m': actual_gap, 'runtime_collision_shape_count': len(shape_prims), 'runtime_shape_rest_offsets_m': shape_rest_offsets, 'runtime_self_collision': self_collision, 'solver_iterations': [args.solver_position_iterations, args.solver_velocity_iterations], 'contact_offset_m': args.contact_offset, 'rest_offset_m': args.rest_offset, 'root_sdf_resolution_requested': args.root_sdf_resolution, 'root_sdf_collision_shapes': root_sdf_shapes, 'velocity_limit_scale': args.velocity_limit_scale, 'hand_velocity_limit_scale': args.hand_velocity_limit_scale, 'hand_velocity_limit_scale_semantics': 'Applied only to 40 hand-axis maxJointVelocity attributes; acceptance remains against the original URDF caps.', 'drive_gain_scale': args.drive_gain_scale, 'arm_drive_gain_scale': args.arm_drive_gain_scale, 'hand_drive_gain_scale': args.hand_drive_gain_scale, 'arm_drive_damping_scale': args.arm_drive_damping_scale, 'hand_drive_damping_scale': args.hand_drive_damping_scale, 'left_thumb_flex_gain_scale': args.left_thumb_flex_gain_scale, 'runtime_drive_settings': drive_settings, 'runtime_velocity_caps': velocity_caps, 'qvel_peak_rad_s_per_joint': dict(zip(names, physx_peak.tolist())), 'qvel_rms_rad_s_per_joint': dict(zip(names, physx_rms.tolist())), 'qpos_fd_peak_rad_s_per_joint': dict(zip(names, qfd_peak.tolist())), 'qpos_fd_rms_rad_s_per_joint': dict(zip(names, qfd_rms.tolist())), 'qvel_limit_fraction_per_joint': {name: float(physx_peak[i]/caps_rad[name]) for i, name in enumerate(names) if name in caps_rad}, 'qvel_limit_fraction_vs_original_urdf_per_joint': {name: float(physx_peak[i]/original_caps_rad[name]) for i, name in enumerate(names) if name in original_caps_rad}, 'qvel_limit_excess_rad_s_per_joint': {name: float(max(0., physx_peak[i]-original_caps_rad[name])) for i, name in enumerate(names) if name in original_caps_rad}, 'tracking_error_peak_rad_per_joint': dict(zip(names, error_peak.tolist())), 'tracking_error_rms_rad_per_joint': dict(zip(names, error_rms.tolist())), 'contact_callback_sample_count': len(contact_events), 'steps_with_contacts': sum(bool(row) for row in contact_rows), 'steps_with_negative_separation': sum(any(e['min_separation_m'] < 0 for e in row) for row in contact_rows), 'minimum_reported_contact_separation_m': min_separation, 'max_target_error_rad': float(np.max(error_peak)), 'rms_target_error_rad': float(np.sqrt(np.mean(target_error**2))), 'accepted': False, 'boundary': 'Simulation-only diagnostic; this setting is not a hardware configuration or acceptance.'}
    if official_policy is None:
        report['official_wuji_assembly_policy'] = {'enabled': False, 'runtime_filtered_pair_count': 0, 'boundary': 'No official Wuji assembly-overlap contact exclusions were applied.'}
    else:
        report['official_wuji_assembly_policy'] = {**official_policy,
                                                   'runtime_filtered_pair_count': len(runtime_filtered_pairs),
                                                   'runtime_filtered_prim_pairs': runtime_filtered_pairs,
                                                   'runtime_readback_passed': len(runtime_filtered_pairs) == 20}
    report['arm_drive_damping_scale'] = args.arm_drive_damping_scale
    report['wrist_distance_feedback'] = {
        'enabled': feedback_state is not None,
        'mode': 'simulation-only bounded q_command residual; nominal q_target and qdot are retained',
        'gain': args.wrist_feedback_gain if feedback_state is not None else None,
        'integral_gain_per_s': args.wrist_feedback_integral_gain if feedback_state is not None else None,
        'max_abs_joint_residual_rad': args.wrist_feedback_max_correction if feedback_state is not None else None,
        'joint_residual_slew_limit_rad_s': args.wrist_feedback_slew_rate if feedback_state is not None else None,
        'controlled_joints': feedback_state['arm_names'] if feedback_state is not None else [],
        'runtime_fk_jacobian_validation': feedback_state['validation'] if feedback_state is not None else None,
        'max_observed_abs_joint_residual_rad': float(np.max(np.abs(np.asarray(command_residual_q)))) if feedback_state is not None else 0.0,
        'max_observed_residual_slew_rad_s': float(np.max(np.abs(np.diff(np.asarray(command_residual_q), axis=0) / np.diff(np.asarray(sim_axis))[:, None]))) if feedback_state is not None and len(sim_axis) > 1 else 0.0,
        'raw_c2_source': str(raw_source_path) if feedback_state is not None else None,
        'raw_c2_source_sha256': raw_source_hash if feedback_state is not None else None,
        'raw_c2_time_semantics': 'desired wrist gap is evaluated from raw-source C2 joint trajectory at mapped original_source_time; execution source_time is virtual reference clock' if feedback_state is not None else None,
        'actual_gap_vs_raw_c2_rms_mm': float(1000 * np.sqrt(np.mean((np.asarray(actual_wrist_gap) - np.asarray(desired_wrist_gap)) ** 2))) if feedback_state is not None else None,
        'actual_gap_vs_raw_c2_peak_mm': float(1000 * np.max(np.abs(np.asarray(actual_wrist_gap) - np.asarray(desired_wrist_gap)))) if feedback_state is not None else None,
        'prediction_semantics': 'measured gap at t-dt plus current Jacobian times measured PhysX qvel times dt; compared with raw-source C2 target at mapped original source time t' if feedback_state is not None else None,
        'diagnostic_only': True,
    }
    report['arm_force_drive_rnea_feedforward'] = {
        'force_drive_enabled': bool(args.arm_force_drive),
        'feedforward_enabled': bool(args.rnea_torque_position_feedforward),
        'mode': 'arm-only force-drive position bias: delta_q = RNEA torque / runtime force-drive stiffness; no direct external effort command',
        'source': str(rnea_feedforward_path) if args.rnea_torque_position_feedforward else None,
        'source_sha256': rnea_feedforward_hash,
        'source_metadata': rnea_metadata if args.rnea_torque_position_feedforward else None,
        'reference_sha256': ref_hash if args.rnea_torque_position_feedforward else None,
        'execution_sha256': execution_hash if args.rnea_torque_position_feedforward else None,
        'raw_source_sha256': raw_source_hash if args.rnea_torque_position_feedforward else None,
        'controlled_joints': [name for name in names if name.startswith(('proximal_', 'elbow_', 'wrist_')) and name.endswith(('_L_Joint', '_R_Joint'))] if args.arm_force_drive else [],
        'force_drive_units': 'Nm for drive stiffness times rad position bias',
        'max_abs_position_bias_rad': args.rnea_feedforward_max_correction if args.rnea_torque_position_feedforward else None,
        'max_observed_abs_position_bias_rad': float(np.max(np.abs(np.asarray(rnea_feedforward_residual_q)[1:]))) if args.rnea_torque_position_feedforward else 0.0,
        'position_bias_per_joint_rad': {name: float(np.max(np.abs(np.asarray(rnea_feedforward_residual_q)[1:, names.index(name)]))) for name in names if name.startswith(('proximal_', 'elbow_', 'wrist_')) and name.endswith(('_L_Joint', '_R_Joint'))} if args.rnea_torque_position_feedforward else {},
        'source_rnea_peak_torque_nm_per_joint': {name: float(np.nanmax(np.abs(np.asarray(rnea_feedforward_tau_q)[1:, names.index(name)]))) for name in names if name.startswith(('proximal_', 'elbow_', 'wrist_')) and name.endswith(('_L_Joint', '_R_Joint'))} if args.rnea_torque_position_feedforward else {},
        'source_rnea_full_trajectory_peak_torque_nm_per_joint': {name: float(np.max(np.abs(rnea_tau[:, names.index(name)]))) for name in names if name.startswith(('proximal_', 'elbow_', 'wrist_')) and name.endswith(('_L_Joint', '_R_Joint'))} if args.rnea_torque_position_feedforward else {},
        'q_command_joint_limit_clip_count': rnea_feedforward_q_limit_clip_count if args.rnea_torque_position_feedforward else 0,
        'authored_max_force_unchanged': bool(args.arm_force_drive),
        'nominal_target_and_velocity_preserved': True,
        'direct_effort_addition': False,
        'diagnostic_only': True,
    }
    report.update({
    'initial_trace': str(args.initial_trace.resolve()) if args.initial_trace is not None else None,
    'initial_state_semantics': 'interpolated q_actual and qvel_actual at source phase; no settling' if initial_state is not None else f'reference q with zero velocity followed by {int(round(.5/dt))} settling steps ({int(round(.5/dt))*dt:.9f}s)',
    'execution': str(execution_path) if execution_path is not None else None,
    'execution_sha256': execution_hash,
    'raw_source': str(raw_source_path) if raw_source_path is not None else None,
    'raw_source_sha256': raw_source_hash,
    'retiming_manifest': str(manifest_path) if execution_path is not None else None,
    'retiming_manifest_sha256': prov.get('retiming_manifest_sha256') if execution_path is not None else None,
    'position_command_semantics': ('nominal q_target plus bounded arm-only RNEA torque/runtime force-drive stiffness position bias; nominal qdot retained' if args.rnea_torque_position_feedforward else ('nominal q_target with nominal qdot velocity target; no command bias' if not args.wrist_distance_feedback else 'nominal q_target plus bounded wrist-distance feedback residual; nominal qdot retained')),
                   'measured_effort_semantics': 'solver-computed joint effort; not a hardware torque or acceptance claim',
                   'applied_effort_peak_abs_per_joint': dict(zip(names, np.nanmax(np.abs(effort_np), axis=0).tolist())),
                   'applied_effort_fraction_at_98pct_maxforce_per_joint': {name: float(np.nanmean(np.abs(effort_np[:, i]) >= .98 * drive_settings[name]['max_force'])) for i, name in enumerate(names) if drive_settings[name]['max_force'] > 0},
                   'measured_effort_peak_abs_per_joint': dict(zip(names, np.nanmax(np.abs(measured_effort_np), axis=0).tolist())),
                   'measured_effort_fraction_at_98pct_maxforce_per_joint': {name: float(np.nanmean(np.abs(measured_effort_np[:, i]) >= .98 * drive_settings[name]['max_force'])) for i, name in enumerate(names) if drive_settings[name]['max_force'] > 0},
                   'physics_hz_nominal': args.physics_hz, 'nominal_step_s': dt, 'full_nominal_steps': full_step_count,
                   'final_physics_step_s': float(final_step_dt if final_step_dt > 0 else dt),
                   'final_step_shortened': bool(final_step_dt > 0),
                   'requested_simulation_duration_s': requested_sim_duration})
    (args.out/'report.json').write_text(json.dumps(report, indent=2))
    print('PHYSICS_RESULT', json.dumps(report), flush=True)
except BaseException as exc:
    (args.out/'failure.json').write_text(json.dumps({'error': repr(exc), 'traceback': traceback.format_exc()}, indent=2))
    print('PHYSICS_FAILURE', traceback.format_exc(), flush=True)
    raise
finally:
    app.close()
