"""Execute repaired source targets at6x action speed in PhysX; no hardware I/O."""
import argparse, datetime, hashlib, json, time
from pathlib import Path
import numpy as np
root = Path(__file__).resolve().parents[1]
source = json.loads((root/'configs/physics.json').read_text())
p = argparse.ArgumentParser(description=__doc__)
p.add_argument('--headless', action='store_true')
p.add_argument('--wall-speed', type=float, default=1)
p.add_argument('--max-seconds', type=float, help='Maximum simulation seconds, starting at source zero')
p.add_argument('--out', type=Path)
a = p.parse_args()
if not np.isfinite(a.wall_speed) or a.wall_speed <= 0:
    raise ValueError('Positive finite wall speed required')
if a.max_seconds is not None and (not np.isfinite(a.max_seconds) or a.max_seconds <= 0):
    raise ValueError('Positive finite maximum simulation seconds required')
a.motion_speed = 6
if source['motion_speed'] !=6 or source['physics_hz'] !=240:
    raise ValueError('Unexpected published physics timing')
a.velocity_limit_scale = float(source['velocity_limit_scale'])
if a.velocity_limit_scale != .98 or source['interpolation'] != 'piecewise linear':
    raise ValueError('Unexpected published 6x controller settings')
a.start_source_seconds = 0
a.max_source_seconds = None if a.max_seconds is None else a.max_seconds*6
a.contact_offset = .001
a.rest_offset = .0002
margin_map = source['per_body_rest_offsets_m']
if any(not np.isfinite(float(v)) or not 0<=float(v)<a.contact_offset for v in margin_map.values()):
    raise ValueError('Invalid per-body rest offset')
a.out = (a.out or root.parent/(root.name+'-runs')/datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f_physics')).resolve()
if a.out == root or root in a.out.parents:
    raise ValueError('Write physical replay records outside the repository')
usd, ref = [root/source[key] for key in ['usd', 'reference']]
input_hashes = {key: hashlib.sha256(path.read_bytes()).hexdigest() for key,path in [('usd',usd),('reference',ref)]}
if any(input_hashes[key] != source[key+'_sha256'] for key in input_hashes):
    raise ValueError('Published model or repaired reference binding changed')
producer_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
d=np.load(ref);axis=d['time'];q=d['q'];names=d['joint_names'].tolist();end=min(float(axis[-1]),a.max_source_seconds or float(axis[-1]));dt=1/240
if not 0<=a.start_source_seconds<end<=float(axis[-1]):raise ValueError('invalid source interval')
if not np.isfinite(q).all() or np.any(np.diff(axis)<=0):raise ValueError('invalid trajectory')
a.out.mkdir(parents=True,exist_ok=False)
from isaacsim import SimulationApp
app=SimulationApp({'headless':a.headless,'width':1600,'height':1000,'active_gpu':0,'multi_gpu':False})
rows=[];actuals=[];times=[];errors=[]
try:
 import omni.ui as ui
 from pxr import Usd,UsdGeom,UsdLux,UsdPhysics,PhysxSchema,PhysicsSchemaTools,UsdUtils,Gf
 from isaacsim.core.api import World
 from isaacsim.core.prims import SingleArticulation
 from isaacsim.core.utils.stage import add_reference_to_stage
 from isaacsim.core.utils.viewports import set_camera_view
 from isaacsim.core.utils.types import ArticulationAction
 world=World(physics_dt=dt,rendering_dt=1/30,stage_units_in_meters=1)
 add_reference_to_stage(str(usd),'/World/Robot');robot=world.scene.add(SingleArticulation('/World/Robot',name='robot'))
 from omni.physx import get_physx_simulation_interface,get_physx_statistics_interface
 from omni.physx.bindings._physx import PhysicsSceneStats
 contact_pairs={}
 def on_contacts(headers,contacts):
  for header in headers:
   if not header.num_contact_data:continue
   pair=' | '.join(sorted([str(PhysicsSchemaTools.intToSdfPath(header.actor0)),str(PhysicsSchemaTools.intToSdfPath(header.actor1))]));item=contact_pairs.setdefault(pair,{'contact_events':0,'min_separation_m':float('inf'),'max_impulse_norm':0.})
   item['contact_events']+=1
   for c in contacts[header.contact_data_offset:header.contact_data_offset+header.num_contact_data]:
    item['min_separation_m']=min(item['min_separation_m'],float(c.separation));item['max_impulse_norm']=max(item['max_impulse_norm'],float(np.linalg.norm(c.impulse)))
 contact_subscription=get_physx_simulation_interface().subscribe_contact_report_events(on_contacts)
 for body in margin_map:
  prim=world.stage.GetPrimAtPath('/World/Robot/'+body)
  if not prim.IsValid() or not prim.HasAPI(UsdPhysics.RigidBodyAPI):raise ValueError('Unknown collision body: '+body)
 # Enable self-contact and tighten the solver without changing geometry.
 for prim in Usd.PrimRange(world.stage.GetPrimAtPath('/World/Robot'),Usd.TraverseInstanceProxies()):
  if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
   api=PhysxSchema.PhysxArticulationAPI.Apply(prim);api.CreateEnabledSelfCollisionsAttr(True);api.CreateSolverPositionIterationCountAttr(32);api.CreateSolverVelocityIterationCountAttr(8)
  if prim.HasAPI(UsdPhysics.CollisionAPI) and not prim.IsInstanceProxy():
   api=PhysxSchema.PhysxCollisionAPI.Apply(prim);api.CreateContactOffsetAttr(a.contact_offset);api.CreateRestOffsetAttr(float(margin_map.get(str(prim.GetPath()).split('/')[3],a.rest_offset)))
 # Preserve authored stiffness/effort bounds and add USD angular damping.
 # This is a simulation controller setting, not a measured hardware gain.
 drives={}
 velocity_caps={}
 massless_overrides={}
 for prim in world.stage.Traverse():
  if prim.HasAPI(UsdPhysics.RigidBodyAPI):PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.)
  if prim.GetName().endswith("_tip") and prim.HasAPI(UsdPhysics.MassAPI):
   api=UsdPhysics.MassAPI(prim)
   if api.GetMassAttr().Get()==0:
    api.GetMassAttr().Set(1e-6);api.GetCenterOfMassAttr().Set(Gf.Vec3f(0));api.GetDiagonalInertiaAttr().Set(Gf.Vec3f(1e-10));massless_overrides[str(prim.GetPath())]={"source_mass":0,"runtime_mass_kg":1e-6,"runtime_diagonal_inertia_kg_m2":[1e-10]*3,"reason":"empty coordinate frame otherwise receives PhysX default 1 kg"}
 for prim in world.stage.Traverse():
  if prim.IsA(UsdPhysics.RevoluteJoint):
   attr=prim.GetAttribute('physxJoint:maxJointVelocity');cap=float(attr.Get());attr.Set(cap*a.velocity_limit_scale);velocity_caps[prim.GetName()]={'authored_deg_s':cap,'runtime_deg_s':cap*a.velocity_limit_scale}
   drive=UsdPhysics.DriveAPI(prim,'angular');kp=float(drive.GetStiffnessAttr().Get());old=float(drive.GetDampingAttr().Get());kd=2*np.sqrt(kp);drive.GetDampingAttr().Set(float(kd));drives[prim.GetName()]={'stiffness':kp,'source_damping':old,'runtime_damping':float(kd),'max_force':float(drive.GetMaxForceAttr().Get()),'drive_type':str(drive.GetTypeAttr().Get())}
 UsdLux.DomeLight.Define(world.stage,'/World/Light').CreateIntensityAttr(550)
 table=UsdGeom.Cube.Define(world.stage,'/World/Table');table.AddTranslateOp().Set(Gf.Vec3d(.8,0,-.475));table.AddScaleOp().Set(Gf.Vec3d(.6,.8,.025));table.CreateDisplayColorAttr([Gf.Vec3f(.10,.14,.18)]);UsdPhysics.CollisionAPI.Apply(table.GetPrim())
 backdrop=UsdGeom.Cube.Define(world.stage,'/World/Backdrop');backdrop.AddTranslateOp().Set(Gf.Vec3d(-.8,0,0));backdrop.AddScaleOp().Set(Gf.Vec3d(.01,3,3));backdrop.CreateDisplayColorAttr([Gf.Vec3f(.025,.04,.06)])
 world.reset();stats=PhysicsSceneStats();get_physx_statistics_interface().get_physx_scene_statistics(UsdUtils.StageCache.Get().GetId(world.stage).ToLongInt(),PhysicsSchemaTools.sdfPathToInt(world.get_physics_context().prim_path),stats);(a.out/'shape_stats.json').write_text(json.dumps({n:getattr(stats,n) for n in dir(stats) if n.startswith('nb_')},indent=2));ids=np.array([robot.get_dof_index(n) for n in names],dtype=np.int32)
 if robot.num_dof!=56 or len(set(ids.tolist()))!=56 or np.any(ids<0):raise ValueError('joint mapping failed')
 controller=robot.get_articulation_controller();robot.set_joint_positions(q[int(np.searchsorted(axis,a.start_source_seconds))],joint_indices=ids);robot.set_joint_velocities(np.zeros(56),joint_indices=ids)
 controller.apply_action(ArticulationAction(joint_positions=q[int(np.searchsorted(axis,a.start_source_seconds))],joint_velocities=np.zeros(56),joint_indices=ids))
 for _ in range(120):world.step(render=False)
 if not a.headless:set_camera_view(np.array([1.85,.72,.65]),np.array([.15,0,.20]))
 state={'play':True};label=None
 if not a.headless:
  window=ui.Window('PhysX bag tracking',width=480,height=150)
  def pause():state['play']=not state['play']
  with window.frame:
   with ui.VStack(spacing=8):
    ui.Label(f'PHYSICS 240 Hz | wall {a.wall_speed:g}x | motion {a.motion_speed:g}x')
    ui.Label('Hands named | body mapping UNCONFIRMED')
    label=ui.Label('Starting...');ui.Button('Pause / Resume',clicked_fn=pause)
 self_collision=[bool(p.GetAttribute('physxArticulation:enabledSelfCollisions').Get()) for p in world.stage.Traverse() if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
 record={'mode':'PhysX position/velocity targets after single initialization','target_interpolation':'piecewise linear','producer_sha256':producer_sha256,'usd_sha256':input_hashes['usd'],'usd':str(usd),'reference_sha256':input_hashes['reference'],'reference':str(ref),'start_source_seconds':a.start_source_seconds,'solver_iterations':[32,8],'contact_offset_m':a.contact_offset,'rest_offset_m':a.rest_offset,'per_body_rest_offsets_m':margin_map,'body_mapping_status':source['body_mapping_status'],'hand_mapping_status':source['hand_mapping_status'],'physics_hz':240,'wall_speed_requested':a.wall_speed,'motion_speed':a.motion_speed,'gravity_m_s2':[0,0,-9.81],'self_collision_enabled':self_collision,'table_collision_enabled':True,'physical_base_to_table_m':.45,'runtime_gain_kp_rad':controller.get_gains()[0][ids].tolist(),'runtime_gain_kd_rad':controller.get_gains()[1][ids].tolist(),'runtime_drive_settings':drives,'runtime_velocity_limit_scale':a.velocity_limit_scale,'runtime_velocity_caps':velocity_caps,'massless_frame_overrides':massless_overrides,'accepted':False,'acceptance_boundary':'Physics tracking diagnostic; collision-pair coverage and hardware behavior unverified'}
 record.update(hardware_execution_authorized=False,physics_stepped=True,acceptance_scope='New physics diagnostic; inspect the new trace before acceptance')
 sim_t=0.;wall_start=time.perf_counter();pause_start=None;paused_total=0.;step=0;next_render=0.;status='running'
 def save(status):
  record.update(status=status,source_time_s=min(a.start_source_seconds+sim_t*a.motion_speed,end),simulation_time_s=sim_t,wall_elapsed_s=time.perf_counter()-wall_start-paused_total,frames_measured=len(actuals),max_error_rad=max(errors,default=0.),rms_error_rad=float(np.sqrt(np.mean((np.asarray(actuals)-np.asarray(rows))**2))) if rows else None)
  tmp=a.out/'report.tmp';tmp.write_text(json.dumps(record,indent=2));tmp.replace(a.out/'report.json')
 while app.is_running() and a.start_source_seconds+sim_t*a.motion_speed<end:
  if not state['play']:
   if pause_start is None:pause_start=time.perf_counter()
   world.render();time.sleep(.02);continue
  if pause_start is not None:paused_total+=time.perf_counter()-pause_start;pause_start=None
  t=min(a.start_source_seconds+sim_t*a.motion_speed,end);i=int(np.clip(np.searchsorted(axis,t,side='right')-1,0,len(axis)-2));fraction=np.clip((t-axis[i])/(axis[i+1]-axis[i]),0,1);target=q[i]*(1-fraction)+q[i+1]*fraction;velocity=(q[i+1]-q[i])/(axis[i+1]-axis[i])*a.motion_speed
  controller.apply_action(ArticulationAction(joint_positions=target,joint_velocities=velocity,joint_indices=ids));world.step(render=False);sim_t+=dt;step+=1
  actual_step=np.asarray(robot.get_joint_positions(joint_indices=ids));rows.append(target.copy());actuals.append(actual_step.copy());times.append(t)
  if sim_t>=next_render:
   world.render();next_render=sim_t+a.wall_speed/30
   actual=np.asarray(robot.get_joint_positions(joint_indices=ids));error=float(np.max(np.abs(actual-target)))
   if not np.isfinite(actual).all():raise RuntimeError('non-finite physics state')
   errors.append(error)
   if label:label.text=f'{t:.1f} / {end:.1f} s | error max {error:.3f} rad'
   if len(errors)%30==0:save('running');print('PHYSICS_PROGRESS',round(t,2),round(error,5),flush=True)
   if error>1.:status='stopped_large_tracking_error';break
   delay=(sim_t/a.wall_speed+wall_start+paused_total)-time.perf_counter()
   if delay>0:time.sleep(delay)
 if status=='running':status='complete' if a.start_source_seconds+sim_t*a.motion_speed>=end else 'closed'
 np.savez_compressed(a.out/'tracking.npz',source_time=np.array(times),q_target=np.array(rows),q_actual=np.array(actuals),joint_names=np.array(names))
 (a.out/'contacts.json').write_text(json.dumps(contact_pairs,indent=2))
 save(status);print('PHYSICS_RESULT',json.dumps({k:v for k,v in record.items() if k!='runtime_drive_settings'}),flush=True)
 if not a.headless:
  if label:label.text=f"{status} | max error {max(errors,default=0):.3f} rad"
  while app.is_running():app.update()
except Exception as exc:
 (a.out/'failure.json').write_text(json.dumps({'error':str(exc)}));raise
finally:app.close()
