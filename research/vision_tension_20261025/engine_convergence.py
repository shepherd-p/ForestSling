"""Engine-step refinement against an independent whole-interval RK4 replay."""
from pathlib import Path
import sys
import json
import argparse
import numpy as np
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT))
from forestlab.sixdof_sling import SixDofSling,mujoco
from independent_rigid_replay import RigidReplay
from sixdof_identification import save


def run(source,out,seconds=2.,start_sec=0.,engine_steps=(.002,.001,.0005),reference_step=.0005):
    out.mkdir(parents=True,exist_ok=False)
    asset=json.loads((source/'ASSET.json').read_text());p=asset['public'];h=asset['evaluator']
    for k in ('cells','half','anchors','nominal_theta'):p[k]=np.asarray(p[k])
    for k in ('com','inertia','theta','weights'):h[k]=np.asarray(h[k])
    raw=np.load(source/'trajectory.npz');start=round(start_sec/.02);end=start+round(seconds/.02)
    states=raw['states'][start:end+1];commands=raw['commands'][start:end]
    independent=RigidReplay(asset);x=independent.from_native(states[0]);reference=[x.copy()]
    for command in commands:
        for _ in range(round(.02/reference_step)):x=independent.step(x,command,reference_step)
        reference.append(x.copy())
    reference=np.asarray(reference);metrics=[]
    for step in engine_steps:
        e=SixDofSling(p,h,dt=step)
        b=independent.b;nq=7*b;nv=6*b;row=states[0]
        e.data.qpos[:]=row[1:1+nq];e.data.qvel[:]=row[1+nq:1+nq+nv]
        e.data.act[:]=row[1+nq+nv:1+nq+nv+4*independent.n];e.data.time=row[0]
        mujoco.mj_forward(e.model,e.data);differences=[]
        for j,command in enumerate(commands):
            e.advance(command,.02)
            actual=independent.from_native(e.record())
            pa,va,qa,wa,ma=independent.unpack(actual);pr,vr,qr,wr,mr=independent.unpack(reference[j+1])
            angle=np.degrees(2*np.arccos(np.clip(abs(np.sum(qa[0]*qr[0])),0.,1.)))
            differences.append([np.linalg.norm(pa[0]-pr[0]),angle,np.linalg.norm(pa[1:]-pr[1:],axis=1).max()])
        d=np.asarray(differences)
        metrics.append(dict(engine_step_sec=step,maximum_payload_com_difference_m=float(d[:,0].max()),
            maximum_payload_angle_difference_deg=float(d[:,1].max()),maximum_aircraft_com_difference_m=float(d[:,2].max())))
        np.savez_compressed(out/f'errors_{step:g}.npz',errors=d)
    save(out/'SUMMARY.json',dict(source=str(source),interval_start_sec=start_sec,interval_duration_sec=seconds,
        independent_step_sec=reference_step,metrics=metrics,no_resets_within_interval=True))
    print(metrics,flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--source',required=True);p.add_argument('--out',required=True)
    p.add_argument('--seconds',type=float,default=2.);p.add_argument('--start-sec',type=float,default=0.)
    p.add_argument('--engine-steps',type=float,nargs='+',default=[.002,.001,.0005])
    p.add_argument('--reference-step',type=float,default=.0005)
    a=p.parse_args();run(Path(a.source).resolve(),Path(a.out).resolve(),a.seconds,a.start_sec,a.engine_steps,a.reference_step)
