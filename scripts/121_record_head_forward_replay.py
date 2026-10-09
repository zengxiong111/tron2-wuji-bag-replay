"""Render one Isaac camera video from a verified head-forward physical replay."""
import argparse
import hashlib
import json
import subprocess
from pathlib import Path

import numpy as np

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--tracking', type=Path, required=True)
parser.add_argument('--usd', type=Path, required=True)
parser.add_argument('--out', type=Path, required=True)
parser.add_argument('--fps', type=int, default=30)
parser.add_argument('--source-start', type=float)
parser.add_argument('--seconds', type=float)
parser.add_argument('--motion-speed',type=float,default=4)
args = parser.parse_args()
if not np.isfinite(args.motion_speed) or args.motion_speed <= 0:
    raise ValueError('Motion speed must be finite and positive')
samples = np.load(args.tracking)
names = samples['joint_names'].tolist()
head_indices = [names.index(n) for n in ['head_yaw_Joint', 'head_pitch_Joint']]
if not np.isfinite(samples['q_actual']).all() or args.fps <= 0:
    raise ValueError('Invalid trace or frame rate')
args.out.mkdir(parents=True, exist_ok=False)

from isaacsim import SimulationApp
app = SimulationApp({'headless': True, 'width': 1280, 'height': 720, 'multi_gpu': False})
try:
    import cv2
    from isaacsim.core.api import World
    from isaacsim.core.prims import SingleArticulation
    from isaacsim.core.simulation_manager import SimulationManager
    from isaacsim.core.utils.stage import add_reference_to_stage
    from isaacsim.sensors.camera import Camera
    from omni.physx import get_physx_interface
    from pxr import Gf, UsdGeom, UsdLux
    from scipy.spatial.transform import Rotation

    world = World(physics_dt=1/240, rendering_dt=1/args.fps, stage_units_in_meters=1)
    add_reference_to_stage(str(args.usd.resolve()), '/World/Robot')
    robot = world.scene.add(SingleArticulation('/World/Robot', name='robot'))
    camera = world.scene.add(Camera('/World/Camera', name='camera', resolution=(1280, 720), frequency=args.fps))
    UsdLux.DomeLight.Define(world.stage, '/World/Light').CreateIntensityAttr(650)
    table = UsdGeom.Cube.Define(world.stage, '/World/Table')
    table.AddTranslateOp().Set(Gf.Vec3d(.8, 0, -.475))
    table.AddScaleOp().Set(Gf.Vec3d(.6, .8, .025))
    table.CreateDisplayColorAttr([Gf.Vec3f(.15, .19, .23)])
    world.reset()
    SimulationManager.enable_fabric(False)
    world.set_simulation_dt(physics_dt=0, rendering_dt=1/args.fps)
    indices = np.array([robot.get_dof_index(n) for n in names])
    target = np.array([.15, 0, -.10])
    eye = target + np.array([2.2, .95, .70])
    forward = (target-eye)/np.linalg.norm(target-eye)
    side = np.cross([0, 0, 1], forward)
    side /= np.linalg.norm(side)
    up = np.cross(forward, side)
    quaternion = Rotation.from_matrix(np.column_stack((forward, side, up))).as_quat()
    camera.set_world_pose(eye, [quaternion[3], *quaternion[:3]], camera_axes='world')
    camera.set_focal_length(2.4)
    camera.set_clipping_range(.001, 100)
    source_start = float(samples['source_time'][0]) if args.source_start is None else args.source_start
    available_end = float(samples['source_time'][-1]) + args.motion_speed/240
    duration = (available_end-source_start)/args.motion_speed if args.seconds is None else args.seconds
    if not float(samples['source_time'][0]) <= source_start < available_end or duration <= 0 or source_start+duration*args.motion_speed > available_end+1e-7:
        raise ValueError('Video interval lies outside the recorded physical trace')
    frame_count = int(round(duration*args.fps))
    writer = cv2.VideoWriter(str(args.out/'head_forward.mp4'), cv2.VideoWriter_fourcc(*'mp4v'), args.fps, (1280, 720))
    if not writer.isOpened():
        raise RuntimeError('Unable to create video')
    max_readback_error = 0.
    for frame in range(frame_count):
        stamp = source_start + frame/args.fps*args.motion_speed
        positions = np.array([np.interp(stamp, samples['source_time'], samples['q_actual'][:, j]) for j in range(56)])
        robot.set_joint_positions(positions, joint_indices=indices)
        world.physics_sim_view.update_articulations_kinematic()
        get_physx_interface().update_transformations(False, True, True, False)
        for _ in range(24 if frame == 0 else 3):
            world.render()
        max_readback_error = max(max_readback_error, float(abs(robot.get_joint_positions(joint_indices=indices)-positions).max()))
        image = np.asarray(camera.get_rgba(), dtype=np.uint8).reshape(720, 1280, 4)
        image = cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)
        cv2.rectangle(image, (0, 0), (1280, 56), (25, 25, 25), -1)
        cv2.putText(image, f'HEAD FORWARD | {args.motion_speed:g}x motion | source {stamp:.2f}s', (20, 36), cv2.FONT_HERSHEY_SIMPLEX, .8, (255, 255, 255), 2)
        writer.write(image)
        if frame in [0, frame_count//2, frame_count-1]:
            cv2.imwrite(str(args.out/f'frame_{frame:04d}.png'), image)
        if frame % args.fps == 0:
            print('RECORDING', frame, '/', frame_count, flush=True)
    writer.release()
    encoded = args.out/'head_forward_h264.mp4'
    subprocess.run(['ffmpeg', '-y', '-loglevel', 'error', '-i', str(args.out/'head_forward.mp4'),
                    '-c:v', 'libx264', '-crf', '20', '-pix_fmt', 'yuv420p',
                    '-movflags', '+faststart', str(encoded)], check=True)
    encoded.replace(args.out/'head_forward.mp4')
    report = {'fps': args.fps, 'frames': frame_count, 'duration_s': frame_count/args.fps, 'codec': 'H264 yuv420p faststart',
              'motion_speed': args.motion_speed, 'source_interval_s': [float(samples['source_time'][0]), float(samples['source_time'][-1])],
              'video_source_interval_s': [source_start, min(available_end, source_start+duration*args.motion_speed)],
              'last_frame_source_time_s': source_start+(frame_count-1)/args.fps*args.motion_speed,
              'head_target_rad': [0, 0], 'actual_head_max_abs_rad': np.max(abs(samples['q_actual'][:, head_indices]), axis=0).tolist(),
              'max_readback_error_rad': max_readback_error, 'physics_stepped_for_capture': False,
              'tracking_sha256': hashlib.sha256(args.tracking.read_bytes()).hexdigest(),
              'usd_sha256': hashlib.sha256(args.usd.read_bytes()).hexdigest(),
              'producer_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
              'boundary': 'Fixed-fps rendering of newly recorded physical states; acceptance belongs to the separate physical trace audit'}
    (args.out/'recording.json').write_text(json.dumps(report, indent=2))
finally:
    app.close()
