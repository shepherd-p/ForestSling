"""Recompute reported errors, CoM estimates and course separation from archives.

This script requires only NumPy. It never starts a simulator or changes a run.
"""
from pathlib import Path
import json,hashlib
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
LAB=ROOT if (ROOT/'runs').is_dir() else ROOT.parent
E=json.loads((ROOT/'supplementary/evidence.json').read_text(encoding='utf-8'))

def read(p):return json.loads(p.read_text(encoding='utf-8'))
def matrices(q):
    q=q/np.linalg.norm(q,axis=-1,keepdims=True);w,x,y,z=np.moveaxis(q,-1,0)
    return np.stack([1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w),
      2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w),
      2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)],axis=-1).reshape(*q.shape[:-1],3,3)

records=[]
cases=[('runs/vision_tension_joint_matrix_20261025_v03/'+r['case'],r) for r in E['matrix']]
cases += [('runs/'+r['case']+'/4_forked_estimated',r['summary']) for r in E['courses']]
for relative,summary in cases:
    folder=LAB/relative;d=np.load(folder/'trajectory.npz');state=d['states'];n=summary['count'];b=n+1
    assert np.isfinite(state).all() and np.all(np.diff(state[:,0])>0)
    err=d['errors'];row=dict(case=relative,transport_executed=bool(len(err)))
    if len(err):
        pos=float(err[:,1].max());angle=float(err[:,2].max())
        assert abs(pos-summary['max_position_error_m'])<1e-10
        assert abs(angle-summary['max_attitude_error_deg'])<1e-10
        row.update(max_position_error_m=pos,max_attitude_error_deg=angle)
    assert state[:,-n:].min()>=summary['min_tension_n']-1e-8
    assert state[:,-n:].max()<=summary['max_tension_n']+1e-8
    asset=read(folder/'ASSET.json');theta=np.asarray(read(folder/'CONTROL_MODEL.json')['theta'])
    com=float(np.linalg.norm(theta[1:4]/theta[0]-(np.asarray(asset['evaluator']['com'])-asset['scan_offset'])))
    expected=summary.get('control_model_com_error_m',summary.get('com_error_m'))
    assert abs(com-expected)<1e-10
    row['control_model_com_error_m']=com
    if summary.get('course'):
        q=state[:,1:1+7*b].reshape(-1,b,7);R=matrices(q[:,1:,3:])
        aircraft_centers=q[:,1:,:3]
        minimum=min(float(np.linalg.norm(aircraft_centers[:,i]-aircraft_centers[:,j],axis=1).min()) for i in range(n) for j in range(i))
        geometry=read(folder/'analysis/METRICS.json')
        assert abs(minimum-geometry['minimum_aircraft_separation_m'])<1e-9
        row['minimum_aircraft_separation_m']=minimum
    row['passed_recomputation']=True;records.append(row)
source_checks=[]
for relative,item in E['source_files'].items():
    path=LAB/relative
    assert path.is_file(),relative
    assert hashlib.sha256(path.read_bytes()).hexdigest()==item['sha256'],relative
    source_checks.append(relative)
out=dict(passed=True,task_attempts=len(records),completed_model_pairs=1,
         source_files_checked=len(source_checks),records=records)
(ROOT/'supplementary/RECOMPUTATION.json').write_text(json.dumps(out,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps({k:v for k,v in out.items() if k!='records'},indent=2))
