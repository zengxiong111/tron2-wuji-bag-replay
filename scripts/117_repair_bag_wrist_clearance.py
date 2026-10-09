"""Smooth symmetric wrist separation with full SE(3) IK and unchanged hand targets."""
import argparse,hashlib,json,time
from pathlib import Path
import numpy as np,pinocchio as pin,coal
from scipy.ndimage import maximum_filter1d,gaussian_filter1d
p=argparse.ArgumentParser();p.add_argument('--safety-margin',type=float,default=.03);p.add_argument('--input',type=Path,required=True);p.add_argument('--out',type=Path,required=True);a=p.parse_args();a.out.mkdir(parents=True,exist_ok=False)
root=Path(__file__).resolve().parents[1];urdf=root/'outputs/robot/tron2_wuji_hand2.urdf';m=pin.buildModelFromUrdf(str(urdf));g=pin.buildGeomFromUrdf(m,str(urdf),pin.GeometryType.COLLISION,package_dirs=[str(root)]);left=[i for i,x in enumerate(g.geometryObjects) if x.name.startswith('l_')];right=[i for i,x in enumerate(g.geometryObjects) if x.name.startswith('r_')]
for i in left:
 for j in right:g.addCollisionPair(pin.CollisionPair(i,j))
data=m.createData();gd=pin.GeometryData(g)
for req in gd.collisionRequests:req.enable_contact=True;req.num_max_contacts=16
s=np.load(a.input);q=s['q'];names=s['joint_names'];idx=np.array([m.idx_qs[m.getJointId(str(n))] for n in names]);armidx=[idx[:7],idx[7:14]];frames=[m.getFrameId('l_wrist'),m.getFrameId('r_wrist')];displacement=np.zeros(len(q));directions=[];start=time.perf_counter()
def intersects():
 pin.computeCollisions(g,gd,False)
 return any(r.isCollision() for r in gd.collisionResults)
for i,row in enumerate(q):
 full=pin.neutral(m);full[idx]=row;pin.forwardKinematics(m,data,full);pin.updateFramePlacements(m,data);pin.updateGeometryPlacements(m,data,g,gd,full)
 direction=data.oMf[frames[0]].translation-data.oMf[frames[1]].translation;distance=float(np.linalg.norm(direction))
 if distance<1e-6:raise ValueError(f'Coincident wrist origins at frame {i}')
 direction/=distance;directions.append(direction)
 if intersects():
  original=[pose.copy() for pose in gd.oMg]
  for magnitude in np.arange(.005,.161,.005):
   for gi in left:gd.oMg[gi]=pin.SE3(original[gi].rotation,original[gi].translation+direction*magnitude/2)
   for gi in right:gd.oMg[gi]=pin.SE3(original[gi].rotation,original[gi].translation-direction*magnitude/2)
   if not intersects():displacement[i]=magnitude;break
  else:raise RuntimeError(f'No separation below 160mm at frame {i}')
 if i%1000==0:print('SEPARATION',i,float(np.max(displacement))*1000,round(time.perf_counter()-start,1),flush=True)
assert np.isfinite(directions).all(), 'invalid wrist separation direction'
# A conservative continuous envelope spreads repairs across nearby source time.
profile=gaussian_filter1d(maximum_filter1d(displacement,size=61,mode='nearest'),sigma=6,mode='nearest')
profile=profile+a.safety_margin*(1-np.exp(-profile/.003))
result=q.copy();position_errors=[];angle_errors=[]
for i,row in enumerate(q):
 if profile[i]<=0:continue
 full=pin.neutral(m);full[idx]=row;pin.forwardKinematics(m,data,full);pin.updateFramePlacements(m,data);targets=[data.oMf[f].copy() for f in frames]
 for side,(frame,active,target) in enumerate(zip(frames,armidx,targets)):
  target.translation+=directions[i]*profile[i]/2*(1 if side==0 else -1)
  for iteration in range(35):
   pin.forwardKinematics(m,data,full);pin.updateFramePlacements(m,data);pose=data.oMf[frame];error=np.r_[target.translation-pose.translation,pin.log3(target.rotation@pose.rotation.T)]
   if np.linalg.norm(error)<1e-6:break
   jac=pin.computeFrameJacobian(m,data,full,frame,pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)[:,active];step=jac.T@np.linalg.solve(jac@jac.T+np.eye(6)*1e-6,error);step=np.clip(step,-.05,.05);full[active]=np.clip(full[active]+step,m.lowerPositionLimit[active]+1e-6,m.upperPositionLimit[active]-1e-6)
  pin.forwardKinematics(m,data,full);pin.updateFramePlacements(m,data);pose=data.oMf[frame];position_errors.append(float(np.linalg.norm(target.translation-pose.translation)));angle_errors.append(float(np.linalg.norm(pin.log3(target.rotation@pose.rotation.T))))
 result[i,:14]=full[idx[:14]]
 if i%1000==0:print('IK',i,flush=True)
assert np.array_equal(result[:,14:],q[:,14:])
if max(position_errors,default=0)>.0002 or max(angle_errors,default=0)>.002:raise RuntimeError('IK failed')
np.savez_compressed(a.out/'reference.npz',**{k:s[k] for k in s.files if k!='q'},q=result)
np.savez_compressed(a.out/'repair_profile.npz',time=s['time'],total_separation_m=profile,direction=np.array(directions))
report={'source_sha256':hashlib.sha256(a.input.read_bytes()).hexdigest(),'urdf_sha256':hashlib.sha256(urdf.read_bytes()).hexdigest(),'output_sha256':hashlib.sha256((a.out/'reference.npz').read_bytes()).hexdigest(),'hand_and_head_targets_unchanged':True,'max_total_wrist_separation_m':float(max(profile)),'changed_rows':int(np.count_nonzero(profile)),'max_joint_delta_rad':float(np.max(np.abs(result-q))),'max_wrist_position_error_m':max(position_errors,default=0),'max_wrist_rotation_error_rad':max(angle_errors,default=0),'physical_base_to_table_m':.45,'smoothing_source_window_s':2.,'safety_margin_m':a.safety_margin,'margin_taper':'1-exp(-profile/0.003); continuous at zero','elapsed_s':time.perf_counter()-start}
# Preserve the original feedback binding across sequential repair stages.
parent_manifest=a.input.parent/'repair.json'
if parent_manifest.is_file():
 parent=json.loads(parent_manifest.read_text())
 if parent['output_sha256']!=report['source_sha256']:raise ValueError('Parent reference binding mismatch')
 report['parent_reference_sha256']=report['source_sha256'];report['source_sha256']=parent['source_sha256']
report['producer_sha256']=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
(a.out/'repair.json').write_text(json.dumps(report,indent=2));print(report,flush=True)
