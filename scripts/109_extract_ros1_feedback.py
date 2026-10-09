"""Extract named bilateral Wuji feedback and explicitly labeled TRON2 motor order."""
import argparse,hashlib,json
from pathlib import Path
from xml.etree import ElementTree as ET
import numpy as np
from rosbags.rosbag1 import Reader
from rosbags.typesys import Stores,get_typestore,get_types_from_msg
from joint_names import LEFT_ARM_JOINT_NAMES,RIGHT_ARM_JOINT_NAMES,LEFT_HAND_JOINT_NAMES,RIGHT_HAND_JOINT_NAMES
BODY=(*LEFT_ARM_JOINT_NAMES,*RIGHT_ARM_JOINT_NAMES,'head_yaw_Joint','head_pitch_Joint')
NAMES=(*BODY,*LEFT_HAND_JOINT_NAMES,*RIGHT_HAND_JOINT_NAMES)
def sha(p):
 h=hashlib.sha256()
 with p.open('rb') as f:
  for b in iter(lambda:f.read(1048576),b''):h.update(b)
 return h.hexdigest()
def main():
 p=argparse.ArgumentParser(description=__doc__)
 for k in ['bag','usd','urdf','out']:p.add_argument('--'+k,type=Path,required=True)
 p.add_argument('--fps',type=float,default=30);a=p.parse_args()
 if a.fps<=0:raise ValueError('fps must be positive')
 times={'body':[],'hands':[]};values={'body':[],'hands':[]};delay={'body':[],'hands':[]};counts={};body_declared=True
 with Reader(a.bag) as r:
  ts=get_typestore(Stores.ROS1_NOETIC)
  con=[c for c in r.connections if c.topic in ['/motor/state','/wuji/hand/state']]
  if {c.topic for c in con}!={'/motor/state','/wuji/hand/state'}:raise ValueError('both feedback topics required')
  for c in con:ts.register(get_types_from_msg(c.msgdef.data,c.msgtype))
  for c,t,raw in r.messages(connections=con):
   m=ts.deserialize_ros1(raw,c.msgtype)
   if c.topic=='/motor/state':
    key='body';q=np.asarray(m.q,dtype=float)
    if len(q)!=16 or m.na!=16:raise ValueError('motor axis count changed')
    if len(m.names):
     if set(m.names)!=set(BODY) or len(set(m.names))!=16:raise ValueError('motor names do not match model')
     q=q[[list(m.names).index(n) for n in BODY]]
    else:body_declared=False
   else:
    key='hands';parts={}
    for hand in m.hand_state:
     names=[n.replace('left_','l_',1).replace('right_','r_',1) for n in hand.names]
     if len(names)!=len(hand.pos) or len(set(names))!=len(names):raise ValueError('hand name/position mismatch')
     for n,v in zip(names,hand.pos,strict=True):
      if n in parts:raise ValueError('duplicate hand joint')
      parts[n]=float(v)
    if set(parts)!=set((*LEFT_HAND_JOINT_NAMES,*RIGHT_HAND_JOINT_NAMES)):raise ValueError('hand joint set changed')
    q=np.array([parts[n] for n in (*LEFT_HAND_JOINT_NAMES,*RIGHT_HAND_JOINT_NAMES)])
   if not np.isfinite(q).all():raise ValueError('non-finite feedback')
   times[key].append(t);values[key].append(q)
   delay[key].append((t-(m.header.stamp.sec*10**9+m.header.stamp.nanosec))*1e-9)
 start=max(x[0] for x in times.values());stop=min(x[-1] for x in times.values())
 grid=np.arange(0,(stop-start)*1e-9,1/a.fps)
 combined=[]
 for k in ['body','hands']:
  stamps=np.asarray(times[k],dtype=np.int64);q=np.asarray(values[k]);dt=np.diff(stamps)
  if np.any(dt<0):raise ValueError('record timestamps move backwards')
  # Keep the final sample if multiple records share a bag timestamp.
  keep=np.r_[dt!=0,True];stamps=stamps[keep];q=q[keep]
  t=(stamps-start)*1e-9;combined.append(np.column_stack([np.interp(grid,t,q[:,j]) for j in range(q.shape[1])]))
  counts[k]={'records':len(times[k]),'duplicates':int(np.count_nonzero(~keep)),'max_record_gap_s':float(np.max(np.diff(stamps))*1e-9),'median_header_delay_s':float(np.median(delay[k])),'min_header_delay_s':float(np.min(delay[k])),'max_header_delay_s':float(np.max(delay[k]))}
 q=np.column_stack(combined)
 joints={j.get('name'):j for j in ET.parse(a.urdf).getroot().findall('joint') if j.get('type')=='revolute'}
 if set(joints)!=set(NAMES):raise ValueError('URDF moving joints differ')
 limits=np.array([[float(joints[n].find('limit').get(k)) for k in ['lower','upper']] for n in NAMES])
 exceed=np.maximum(limits[:,0]-q,q-limits[:,1]);violations={n:{'frames':int(np.count_nonzero(exceed[:,i]>1e-3)),'max_excess_rad':float(max(0,np.max(exceed[:,i])))} for i,n in enumerate(NAMES) if np.max(exceed[:,i])>1e-3}
 a.out.mkdir(parents=True,exist_ok=False)
 ref=a.out/'feedback_30hz.npz';np.savez_compressed(ref,time=grid,q=q,joint_names=np.array(NAMES),bag_start_ns=np.int64(start))
 report={'bag':str(a.bag.resolve()),'bag_sha256':sha(a.bag),'usd':str(a.usd.resolve()),'usd_sha256':sha(a.usd),'urdf_sha256':sha(a.urdf),'reference':str(ref.resolve()),'reference_sha256':sha(ref),'duration_s':float(grid[-1]),'frames':len(grid),'fps':a.fps,'joint_names':NAMES,'sources':{'body':'/motor/state q','hands':'/wuji/hand/state hand_state.pos'},'source_counts':counts,'body_names_declared':body_declared,'body_mapping_status':'source_declared' if body_declared else 'candidate_unconfirmed','hand_mapping_status':'source_declared_names; left_ -> l_, right_ -> r_','time_basis':'common bag receipt timestamps; linear interpolation; no extrapolation','limit_violations':violations,'position_modified_or_clipped':False,'replay_mode':'kinematic recorded feedback; no physics tracking or world-pose IK'}
 (a.out/'provenance.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n');print(json.dumps(report,ensure_ascii=False,indent=2))
if __name__=='__main__':main()
