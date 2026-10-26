"""Independent replay around high roll, rapid loading and closest approach."""
from pathlib import Path
import sys,json
import numpy as np
ROOT=Path(__file__).resolve().parents[1];LAB=ROOT.parent
sys.path[:0]=[str(LAB),str(LAB/'.deps/markerless'),str(LAB/'research/vision_tension_20261025')]
from engine_convergence import run
from scipy.spatial.transform import Rotation
from forestlab.tension_passage import box_sdf

def main():
    source=LAB/'runs/large_pose_course_20261026_v01/4_forked_estimated'
    raw=np.load(source/'trajectory.npz')['states'];a=json.loads((source/'ASSET.json').read_text(encoding='utf-8'))
    p=json.loads((source/'PROTOCOL.json').read_text(encoding='utf-8'));n=4;b=5
    qp=raw[:,1:1+7*b].reshape(-1,b,7);r=Rotation.from_quat(qp[:,0,3:][:,[1,2,3,0]]).as_euler('xyz',degrees=True)
    motion=float(json.loads((source/'SUMMARY.json').read_text(encoding='utf-8'))['motion_start_sec'])
    lo=np.searchsorted(raw[:,0],motion+1);hi=len(raw)-51
    kroll=lo+np.argmax(abs(r[lo:hi,0]));ktension=lo+np.argmax(abs(np.diff(raw[lo:hi,-n:],axis=0)).max(1))
    clearance=np.full(len(raw),np.inf)
    for box in p['obstacles']:clearance=np.minimum(clearance,box_sdf(qp[:,1:,:3],box).min(1)-.28)
    kgap=lo+np.argmin(clearance[lo:hi]);allrows=[]
    out=ROOT/'supplementary/review_20261027/independent_replay';out.mkdir(exist_ok=True)
    for name,k in [('high_roll',kroll),('tension_transient',ktension),('minimum_clearance',kgap)]:
        start=round(float(raw[k,0]-1)/.02)*.02;folder=out/name
        if not (folder/'SUMMARY.json').exists():run(source,folder,2.,start,(.001,.0005,.00025),.00025)
        s=json.loads((folder/'SUMMARY.json').read_text(encoding='utf-8'))
        s.update(event=name,event_time_sec=float(raw[k,0]),event_roll_deg=float(r[k,0]),
                 event_aircraft_envelope_clearance_m=float(clearance[k]))
        allrows.append(s);(out/'RESULTS.json').write_text(json.dumps(allrows,indent=2)+'\n',encoding='utf-8')
        print(name,s['metrics'],flush=True)

if __name__=='__main__':main()
