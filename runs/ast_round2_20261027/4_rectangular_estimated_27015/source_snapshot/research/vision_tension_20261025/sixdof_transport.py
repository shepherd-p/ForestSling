"""Continuous pose/translation after sensor-only identification, N=3/4."""
from pathlib import Path
import sys
import argparse
import json
import time
from types import SimpleNamespace
import numpy as np
ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT),str(ROOT/'.deps/submission_review'),str(ROOT/'.deps/markerless')]
from forestlab.sixdof_sling import SixDofSling,payload_asset,rotation,rpy
from forestlab.inertial_identification import fit,decode,skew
from forestlab.tension_transport import TensionPoseServo,vee
from sixdof_identification import save


def exp(v):
    a=np.linalg.norm(v); K=skew(v)
    return np.eye(3)+(np.sin(a)/a if a>1e-8 else 1.)*K+((1-np.cos(a))/a**2 if a>1e-8 else .5)*K@K


def log(R):
    a=np.arccos(np.clip((np.trace(R)-1)/2,-1.,1.))
    return (.5 if a<1e-8 else a/(2*np.sin(a)))*vee(R-R.T)


def reference(t,p0,R0,duration=8.,distance=4.,angle=(25.,10.,30.)):
    u=np.clip(t/duration,0.,1.)
    s=10*u**3-15*u**4+6*u**5
    ds=(30*u**2-60*u**3+30*u**4)/duration if 0.<t<duration else 0.
    dds=(60*u-180*u**2+120*u**3)/duration**2 if 0.<t<duration else 0.
    goal=rpy(np.radians(angle)); axis=log(goal@R0.T)
    direction=np.array([distance,0.,0.])
    return SimpleNamespace(position=p0+direction*s,velocity=direction*ds,acceleration=direction*dds,
        rotation=exp(axis*s)@R0,omega_world=axis*ds,alpha_world=axis*dds)


def run(out,seconds=8.,seed=2501):
    out.mkdir(parents=True,exist_ok=False);results=[]
    save(out/'PROTOCOL.json',dict(seed=seed,identification_seconds=16.,motion_seconds=seconds,
        distance_m=4.,goal_rpy_deg=[25,10,30],development=True,
        feedback='Noisy calibrated pose/rate telemetry and independent noisy tension sensors; image pose not used yet',
        gates=dict(max_attitude_error_deg=5.,max_position_error_m=.10,min_tension_n=.5,max_tension_n=9.,
                   max_aircraft_tilt_deg=40.,max_motor_thrust_n=7.5,no_contact=True)))
    for n in (3,4):
        for shape in ('rectangular','irregular'):
            public,hidden=payload_asset(shape,n,'offset_z',seed)
            e=SixDofSling(public,hidden,seed=seed)
            start=e.observation(0.)['uav_position'].copy()
            rows=[e.observation()];states=[e.record()];commands=[]
            for k in range(800):
                f=e.excitation_force(start,k*.02,amplitude=.6)
                motors=e.motor_command(f);e.advance(motors,.02)
                rows.append(e.observation());states.append(e.record());commands.append(motors)
            ident=fit(rows,public['anchors'],window_sec=.4,tension_floor=.01)
            if not ident['activated']: raise RuntimeError('Refuse transport without an activated estimate')
            d=decode(ident['theta']);first=e.observation();p0=first['position'].copy();R0=first['rotation'].copy()
            servo=TensionPoseServo(public['anchors'],ident['theta'],rest_length=e.rest_length,
                stiffness=e.stiffness,damping=e.damping,uav_mass=e.uav_mass,
                pressure=1.25,acceleration_limit=2.5,speed_limit=1.6,attitude_gain=2.,angular_gain=1.75)
            # Warm start from measured cable forces, not a hidden equilibrium.
            delta=first['uav_position']-first['position']-public['anchors']@R0.T
            servo.allocator.previous=delta/np.linalg.norm(delta,axis=1)[:,None]*first['tension'][:,None]
            errors=[];refs=[];timings=[];reason=None;begin=len(states)-1
            movement_tension=[];movement_speed=[]
            for k in range(round((seconds+2.)/.02)):
                obs=e.observation();target=reference((k+1)*.02,p0,R0,seconds)
                tick=time.perf_counter()
                try:f,info=servo.command(obs,e.uav_velocities(),target,.02)
                except RuntimeError as error:reason=str(error);break
                motors=e.motor_command(f);timings.append(time.perf_counter()-tick)
                e.advance(motors,.02);row=e.observation(0.)
                angle=np.degrees(np.linalg.norm(log(row['rotation']@target.rotation.T)))
                errors.append([e.data.time,float(np.linalg.norm(row['position']-target.position)),angle])
                movement_tension.append(row['tension']);movement_speed.append(np.linalg.norm(row['velocity']))
                states.append(e.record());commands.append(motors);refs.append(np.r_[target.position,target.rotation.ravel()])
                if errors[-1][1]>1.5:reason='position_divergence';break
            errors=np.asarray(errors);ten=np.asarray(movement_tension)
            result=dict(count=n,shape=shape,complete=reason is None,reason=reason,
                com_error_m=float(np.linalg.norm(d['com']-hidden['com'])),
                maximum_position_error_m=float(errors[:,1].max()),maximum_attitude_error_deg=float(errors[:,2].max()),
                movement_minimum_tension_n=float(ten.min()),movement_maximum_tension_n=float(ten.max()),
                whole_episode_minimum_tension_n=e.minimum_tension,whole_episode_maximum_tension_n=e.maximum_tension,
                maximum_aircraft_tilt_deg=e.maximum_tilt,maximum_motor_thrust_n=e.maximum_motor,
                collision_steps=e.collision_steps,saturation_intervals=e.motor_saturation_intervals,
                maximum_payload_speed_m_s=float(max(movement_speed)),
                controller_p95_ms=float(np.quantile(timings,.95)*1000))
            result['passed']=bool(reason is None and errors[:,1].max()<=.10 and errors[:,2].max()<=5.
                and e.minimum_tension>.5 and e.maximum_tension<9. and e.maximum_tilt<=40.
                and e.maximum_motor<=7.5 and e.collision_steps==0)
            stem=f'{n}_{shape}';save(out/(stem+'_IDENTIFICATION.json'),ident)
            save(out/(stem+'_ASSET.json'),dict(public=public,evaluator=hidden))
            (out/(stem+'_model.xml')).write_text(e.xml,encoding='utf-8')
            np.savez_compressed(out/(stem+'.npz'),states=states,commands=commands,errors=errors,
                references=refs,motion_start=begin)
            results.append(result);save(out/'RESULTS.json',results)
            print(json.dumps(result,ensure_ascii=True),flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--out',required=True);p.add_argument('--seconds',type=float,default=8.)
    a=p.parse_args();run(Path(a.out).resolve(),a.seconds)
