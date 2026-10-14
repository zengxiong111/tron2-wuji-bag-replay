"""Isaac recorded-path playback; no new physics integration or hardware I/O."""
import argparse
import datetime
import hashlib
import json
from pathlib import Path
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--headless', action='store_true')
parser.add_argument('--out', type=Path)
parser.add_argument('--max-seconds', type=float)
args = parser.parse_args()
binding = json.loads((ROOT/'configs/trajectory.json').read_text())
reference, usd = [ROOT/binding[key] for key in ('trajectory', 'usd')]
sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
if sha(reference) != binding['trajectory_sha256'] or sha(usd) != binding['usd_sha256']:
    raise ValueError('Published trajectory/model binding changed')
data = np.load(reference, allow_pickle=False)
axis, q, names = data['time'], data['q'], data['joint_names'].tolist()
if q.shape != (len(axis), 56) or not np.isfinite(q).all() or not np.all(np.diff(axis) > 0):
    raise ValueError('Invalid trajectory')
if args.max_seconds is not None and (not np.isfinite(args.max_seconds) or args.max_seconds <= 0):
    raise ValueError('Positive finite --max-seconds required')
end = min(float(axis[-1]), args.max_seconds) if args.max_seconds else float(axis[-1])
output = (args.out or ROOT.parent/(ROOT.name+'-runs')/datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f')).resolve()
if output == ROOT or ROOT in output.parents:
    raise ValueError('Write replay records outside the repository')
output.mkdir(parents=True, exist_ok=False)

from isaacsim import SimulationApp
app = SimulationApp({'headless': args.headless, 'width': 1280, 'height': 800, 'multi_gpu': False})
report = {'trajectory_sha256': sha(reference), 'usd_sha256': sha(usd),
          'producer_sha256': sha(Path(__file__)), 'mode': 'recorded-path kinematic playback',
          'interpolation': 'piecewise linear in execution seconds', 'file_time_multiplier': 1,
          'effective_bag_speed': binding['effective_bag_speed'], 'physics_stepped': False, 'physical_base_to_table_m': .45,
          'hardware_execution_authorized': False, 'status': 'starting'}
try:
    from isaacsim.core.api import World
    from isaacsim.core.prims import SingleArticulation
    from isaacsim.core.simulation_manager import SimulationManager
    from isaacsim.core.utils.stage import add_reference_to_stage
    from isaacsim.core.utils.viewports import set_camera_view
    from omni.physx import get_physx_interface
    from pxr import Gf, UsdGeom, UsdLux

    world = World(physics_dt=1/240, rendering_dt=1/30, stage_units_in_meters=1)
    add_reference_to_stage(str(usd), '/World/Robot')
    robot = world.scene.add(SingleArticulation('/World/Robot', name='robot'))
    table = UsdGeom.Cube.Define(world.stage, '/World/Table')
    table.AddTranslateOp().Set(Gf.Vec3d(.8, 0, -.475))
    table.AddScaleOp().Set(Gf.Vec3d(.6, .8, .025))
    table.CreateDisplayColorAttr([Gf.Vec3f(.15, .19, .23)])
    UsdLux.DomeLight.Define(world.stage, '/World/Light').CreateIntensityAttr(650)
    world.reset()
    SimulationManager.enable_fabric(False)
    world.set_simulation_dt(physics_dt=0, rendering_dt=1/30)
    indices = np.array([robot.get_dof_index(n) for n in names])
    if len(set(indices)) != 56:
        raise ValueError('Model joint mapping is not one-to-one')
    set_camera_view(eye=np.array([2.35, .95, .60]), target=np.array([.15, 0, .20]))
    stamps = np.r_[np.arange(float(axis[0]), end, 1/30), end]
    started = time.perf_counter()
    maximum_error = 0.
    frames = 0
    rendered_drift = {}
    first_transforms = {}
    tracked = ['wrist_roll_L_Link', 'wrist_roll_R_Link', 'l_index_finger_distal', 'r_index_finger_distal']
    for stamp in stamps:
        if not app.is_running():
            break
        target = np.array([np.interp(stamp, axis, q[:, j]) for j in range(56)])
        robot.set_joint_positions(target, joint_indices=indices)
        world.physics_sim_view.update_articulations_kinematic()
        get_physx_interface().update_transformations(False, True, True, False)
        for _ in range(3):
            world.render()
        maximum_error = max(maximum_error, float(abs(robot.get_joint_positions(joint_indices=indices)-target).max()))
        cache = UsdGeom.XformCache()
        for name in tracked:
            prim = world.stage.GetPrimAtPath('/World/Robot/'+name)
            if not prim.IsValid():
                raise ValueError('Missing verification link '+name)
            matrix = np.array(cache.GetLocalToWorldTransform(prim))
            first_transforms.setdefault(name, matrix.copy())
            rendered_drift[name] = max(rendered_drift.get(name, 0.), float(abs(matrix-first_transforms[name]).max()))
        frames += 1
        if not args.headless:
            time.sleep(max(0., float(stamp)-time.perf_counter()+started))
    report.update(status='complete' if frames == len(stamps) else 'closed', frames=frames,
                  last_execution_time_s=float(stamps[frames-1]) if frames else None,
                  maximum_joint_readback_error_rad=maximum_error, rendered_link_matrix_max_delta=rendered_drift)
    if maximum_error > 1e-5:
        raise RuntimeError('Recorded-path joint readback mismatch')
    if end >= 1 and not any(value > 1e-5 for value in rendered_drift.values()):
        raise RuntimeError('Rendered links did not change with the trajectory')
except Exception as exc:
    report.update(status='failed', error=str(exc))
    raise
finally:
    (output/'report.json').write_text(json.dumps(report, indent=2)+'\n')
    app.close()
print(json.dumps(report, indent=2))
