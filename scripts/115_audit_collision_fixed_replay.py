"""Read-only Coal mesh audit of targets and recorded PhysX joint snapshots."""
import argparse,hashlib,importlib.util,json,time,sys
from pathlib import Path
import numpy as np
import pinocchio as pin
import coal
from pxr import Usd,UsdPhysics,UsdGeom
root=Path(__file__).resolve().parents[1];folder=root/'outputs/bag_feedback_20261008_adapter';parser=argparse.ArgumentParser();parser.add_argument('--input',type=Path,required=True);parser.add_argument('--out',type=Path,required=True);parser.add_argument('--q-key',default='q_actual');parser.add_argument('--time-key',default='source_time');parser.add_argument('--midpoints',action='store_true');a=parser.parse_args();out=a.out;out.mkdir(parents=True,exist_ok=False)
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
import collision_utils as api
model=pin.buildModelFromUrdf(str(root/'outputs/robot/tron2_wuji_hand2.urdf'));geo=pin.buildGeomFromUrdf(model,str(root/'outputs/robot/tron2_wuji_hand2.urdf'),pin.GeometryType.COLLISION,package_dirs=[str(root)])
coverage=api.collision_coverage(root/'outputs/robot/tron2_wuji_hand2.urdf',geo);summary,exclusions=api.build_pairs(model,geo)
robot_geometry_count=geo.ngeoms;table=pin.GeometryObject('table',0,pin.SE3(np.eye(3),np.array([.8,0,-.475])),coal.Box(1.2,1.6,.05));table_id=geo.addGeometryObject(table)
for i in range(table_id):geo.addCollisionPair(pin.CollisionPair(i,table_id))
data=model.createData();gd=pin.GeometryData(geo)
for req in gd.collisionRequests:req.enable_contact=True;req.num_max_contacts=16
sign=api.verify_penetration_depth_sign()
usd=root/'outputs/robot/tron2_wuji_hand2.usd';stage=Usd.Stage.Open(str(usd));usd_meshes=[]
for prim in Usd.PrimRange(stage.GetDefaultPrim(),Usd.TraverseInstanceProxies()):
 if prim.IsA(UsdGeom.Mesh) and prim.HasAPI(UsdPhysics.CollisionAPI):usd_meshes.append({'path':str(prim.GetPath()),'enabled':UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get(),'approximation':str(UsdPhysics.MeshCollisionAPI(prim).GetApproximationAttr().Get())})
settings={'usd_sha256':sha(usd),'urdf_sha256':sha(root/'outputs/robot/tron2_wuji_hand2.urdf'),'robot_collision_geometry_count':robot_geometry_count,'coverage':coverage,'pairs':{'robot':summary,'table':table_id,'total':len(geo.collisionPairs)},'table_center_m':[.8,0,-.475],'table_size_m':[1.2,1.6,.05],'physical_base_above_table_m':.45,'contact_samples_per_pair':16,'raw_depth_sign_test':sign,'usd_self_collision_enabled':[p.GetAttribute('physxArticulation:enabledSelfCollisions').Get() for p in stage.Traverse() if p.HasAPI(UsdPhysics.ArticulationRootAPI)],'usd_collider_meshes':usd_meshes,'exclusions':exclusions,'boundary':'Discrete triangle-mesh intersection/contact-feature depth. Not exact solid clearance, PhysX convex collider equivalence, CCD or collision response. Adjacent joints and same-rigid meshes excluded; adapter seating assessed separately.'}
parsed=UsdPhysics.LoadUsdPhysicsFromRange(stage,[stage.GetDefaultPrim().GetPath()])
settings['usd_parser_shapes']={str(kind):[str(path) for path in entries[0]] for kind,entries in parsed.items() if 'Shape' in str(kind)}
settings['usd_collision_api_non_gprim']=[str(p.GetPath()) for p in Usd.PrimRange(stage.GetDefaultPrim(),Usd.TraverseInstanceProxies()) if p.HasAPI(UsdPhysics.CollisionAPI) and not p.IsA(UsdGeom.Gprim)]
settings['usd_parser_boundary']='OpenUSD core physics parser; separate from a PhysX runtime contact test. The original Xform CollisionAPI declarations are unrecognized shape types.'
(out/'coverage.json').write_text(json.dumps(settings,indent=2))
def run(label,path,qkey,tkey,midpoints=False):
 d=np.load(path);q=d[qkey];t=d[tkey];names=d['joint_names'].tolist();indices=np.array([model.idx_qs[model.getJointId(n)] for n in names]);assert len(set(indices))==56
 if midpoints:
  qq=np.empty((len(q)*2-1,q.shape[1]));tt=np.empty(len(qq));qq[::2]=q;qq[1::2]=(q[:-1]+q[1:])/2;tt[::2]=t;tt[1::2]=(t[:-1]+t[1:])/2;q,t=qq,tt
 maxima=[];counts=[];pairs={};families={};start=time.perf_counter()
 for row,(positions,stamp) in enumerate(zip(q,t,strict=True)):
  full=pin.neutral(model);full[indices]=positions;pin.computeCollisions(model,data,geo,gd,full,False);maxdepth=0.;count=0
  for pair,result in zip(geo.collisionPairs,gd.collisionResults,strict=True):
   if not result.isCollision():continue
   contacts=result.getContacts()
   if not contacts:continue
   contact=min(contacts,key=lambda c:float(c.penetration_depth));depth=max(0.,-float(contact.penetration_depth))
   if depth<=1e-4:continue
   first=geo.geometryObjects[pair.first].name;second=geo.geometryObjects[pair.second].name;family='table' if second=='table' else api.pair_family(first,second);key=first+' | '+second
   maxdepth=max(maxdepth,depth);count+=1
   item=pairs.setdefault(key,{'first':first,'second':second,'family':family,'sample_count':0,'max_depth_mm':0.,'worst_source_time_s':None,'worst_q':None})
   item['sample_count']+=1
   if depth*1000>item['max_depth_mm']:
    item.update(max_depth_mm=depth*1000,worst_source_time_s=float(stamp),worst_q=positions.tolist(),nearest_point_first=np.array(contact.getNearestPoint1()).tolist(),nearest_point_second=np.array(contact.getNearestPoint2()).tolist())
   fam=families.setdefault(family,{'events':0,'max_depth_mm':0.});fam['events']+=1;fam['max_depth_mm']=max(fam['max_depth_mm'],depth*1000)
  maxima.append(maxdepth*1000);counts.append(count)
  if row%500==0:print(label,row,'/',len(q),'max_mm',round(max(maxima),3),'elapsed',round(time.perf_counter()-start,1),flush=True)
 report={'label':label,'input_sha256':sha(path),'q_dataset':qkey,'samples':len(q),'midpoints':midpoints,'source_duration_s':float(t[-1]-t[0]),'samples_with_penetration_over_0_1mm':int(np.count_nonzero(counts)),'samples_with_penetration_over_1mm':int(np.count_nonzero(np.array(maxima)>1)),'maximum_depth_mm':max(maxima),'families':families,'pairs_sorted':sorted(pairs.values(),key=lambda p:p['max_depth_mm'],reverse=True),'elapsed_s':time.perf_counter()-start}
 (out/(label+'.json')).write_text(json.dumps(report,indent=2));np.savez_compressed(out/(label+'_samples.npz'),source_time=t,max_depth_mm=np.array(maxima),penetrating_pairs=np.array(counts));print(label,'DONE',len(q),report['maximum_depth_mm'],flush=True)
 return report
report=run('checked',a.input,a.q_key,a.time_key,a.midpoints)
(out/'gate.json').write_text(json.dumps({'passed':report['maximum_depth_mm']<=.1,'threshold_mm':.1,'maximum_depth_mm':report['maximum_depth_mm'],'families':report['families'],'input_sha256':report['input_sha256']},indent=2))

if report['maximum_depth_mm']>.1:raise SystemExit(2)
