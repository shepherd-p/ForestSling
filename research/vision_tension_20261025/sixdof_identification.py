"""Sensor-only N=3/4 inertial identification on a motor-driven free-body plant."""
from pathlib import Path
import sys
import argparse
import json
import time
import numpy as np
ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT),str(ROOT/'.deps/submission_review')]
from forestlab.sixdof_sling import SixDofSling,payload_asset
from forestlab.inertial_identification import fit,decode


def save(path,obj):
    def conv(v):
        if isinstance(v,np.ndarray): return v.tolist()
        if isinstance(v,np.generic): return v.item()
        raise TypeError(type(v).__name__)
    Path(path).write_text(json.dumps(obj,default=conv,indent=2,allow_nan=False),encoding='utf-8')


def run(out,seconds=16.,seed=2501):
    out.mkdir(parents=True,exist_ok=False)
    results=[]
    for n in (3,4):
        for shape in ('rectangular','irregular'):
            public,hidden=payload_asset(shape,n,'offset_z',seed)
            e=SixDofSling(public,hidden,seed=seed)
            start=e.observation(0.)['uav_position'].copy()
            rows=[e.observation()]; states=[e.record()]; commands=[]; timings=[]
            for k in range(round(seconds/.02)):
                tick=time.perf_counter()
                f=e.excitation_force(start,k*.02,amplitude=.6)
                motors=e.motor_command(f); timings.append(time.perf_counter()-tick)
                e.advance(motors,.02)
                rows.append(e.observation()); states.append(e.record()); commands.append(motors)
            ident=fit(rows,public['anchors'],window_sec=.4,tension_floor=.01)
            d=decode(ident['theta'])
            stem=f'{n}_{shape}'
            result=dict(count=n,shape=shape,activated=ident['activated'],rank=ident['rank'],
                mass_relative_error=float(abs(d['mass']/hidden['mass']-1)),
                com_error_m=float(np.linalg.norm(d['com']-hidden['com'])),estimated_com=d['com'],true_com=hidden['com'],
                condition=ident['normalized_condition'],minimum_omega_information=ident['minimum_omega_information'],
                minimum_tension_n=e.minimum_tension,maximum_tension_n=e.maximum_tension,
                minimum_extension_m=e.minimum_extension,maximum_aircraft_tilt_deg=e.maximum_tilt,
                maximum_motor_thrust_n=e.maximum_motor,collision_steps=e.collision_steps,
                motor_saturation_intervals=e.motor_saturation_intervals,
                controller_p95_ms=float(np.quantile(timings,.95)*1000))
            results.append(result)
            save(out/(stem+'_IDENTIFICATION.json'),ident);save(out/(stem+'_SENSORS.json'),rows)
            save(out/(stem+'_ASSET.json'),dict(public=public,evaluator=hidden))
            (out/(stem+'_model.xml')).write_text(e.xml,encoding='utf-8')
            np.savez_compressed(out/(stem+'.npz'),states=states,commands=commands)
            save(out/'RESULTS.json',results)
            print(json.dumps(result,default=lambda x:x.tolist(),ensure_ascii=True),flush=True)
    save(out/'PROTOCOL.json',dict(kind='development_motor_driven_sensor_identification',seed=seed,seconds=seconds,
        sensor_source='Calibrated noisy pose/rate telemetry and independent cable tension channel; no image-based pose in this test',
        physics='MuJoCo free payload and free aircraft, 4 motor-site forces per UAV, 25ms motor dynamics, unilateral spring-damper cables',
        sample_dt_sec=.02,physics_dt_sec=.002,initialization='Same nominal geometry and 1.05kg equilibrium in every hidden ballast case'))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--out',required=True);p.add_argument('--seconds',type=float,default=16.)
    a=p.parse_args();run(Path(a.out).resolve(),a.seconds)
