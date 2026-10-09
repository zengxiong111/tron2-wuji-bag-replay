"""Reduce feedback high-frequency noise in a separately bound replay reference."""
import argparse,hashlib,json
from pathlib import Path
import numpy as np
from scipy.signal import butter,sosfiltfilt
import pinocchio as pin
p=argparse.ArgumentParser();p.add_argument('--input',type=Path,required=True);p.add_argument('--out',type=Path,required=True);p.add_argument('--cutoff-source-hz',type=float,default=1.);p.add_argument('--motion-speed',type=float,default=5);p.add_argument('--head-forward',action='store_true');a=p.parse_args();a.out.mkdir(parents=True,exist_ok=False)
if not np.isfinite(a.motion_speed) or a.motion_speed<=0:raise ValueError('Motion speed must be finite and positive')
s=np.load(a.input);t=s['time'];q=s['q'];fs=1/float(np.diff(t).mean())
if not np.isfinite(q).all() or not np.allclose(np.diff(t),1/fs,atol=1e-7,rtol=0):raise ValueError('Expected finite, uniformly resampled feedback')
if not np.isfinite(a.cutoff_source_hz) or not 0<a.cutoff_source_hz<fs/2:raise ValueError('Cutoff must be positive and below Nyquist')
filtered=sosfiltfilt(butter(4,a.cutoff_source_hz,fs=fs,output='sos'),q,axis=0)
m=pin.buildModelFromUrdf(str(Path(__file__).resolve().parents[1]/'outputs/robot/tron2_wuji_hand2.urdf'));indices=np.array([m.idx_qs[m.getJointId(str(n))] for n in s['joint_names']]);filtered=np.clip(filtered,m.lowerPositionLimit[indices]+.005,m.upperPositionLimit[indices]-.005)
if a.head_forward:
 for name in ['head_yaw','head_pitch']:
  matches=[i for i,n in enumerate(s['joint_names']) if name in str(n)]
  if len(matches)!=1:raise ValueError('head mapping ambiguous')
  filtered[:,matches[0]]=0
np.savez_compressed(a.out/'reference.npz',**{k:s[k] for k in s.files if k!='q'},q=filtered)
parent_path=a.input.parent/'repair.json';parent=json.loads(parent_path.read_text()) if parent_path.exists() else {'source_sha256':hashlib.sha256(a.input.read_bytes()).hexdigest()};report={'source_sha256':parent['source_sha256'],'parent_reference_sha256':hashlib.sha256(a.input.read_bytes()).hexdigest(),'output_sha256':hashlib.sha256((a.out/'reference.npz').read_bytes()).hexdigest(),'method':'zero-phase fourth-order Butterworth followed by C2 cubic interpolation during replay','cutoff_source_hz':a.cutoff_source_hz,'cutoff_simulation_hz':a.cutoff_source_hz*a.motion_speed,'max_target_change_rad':float(np.max(abs(filtered-q))),'rms_target_change_rad':float(np.sqrt(np.mean((filtered-q)**2))),'motion_speed':a.motion_speed,'head_forward':a.head_forward,'base_table_gap_m':.45,'body_mapping_status':'candidate_unconfirmed','producer_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest()};(a.out/'repair.json').write_text(json.dumps(report,indent=2));print(report)
