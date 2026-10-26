"""Causal observation-prefix study; evaluator parameters only score final fits."""
from pathlib import Path
import argparse
import json
import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
from matplotlib import pyplot as plt

ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT),str(ROOT/'.deps/markerless'),str(ROOT/'.deps/submission_review')]
from vision_closed_loop import lagged_kinematic_rows
from forestlab.shape_tension_identification import fit_shape_supported
from forestlab.inertial_identification import decode,parameters,wrench_matrix
from forestlab.tension_passage import PassagePlan
from sixdof_identification import save


def run(root,out):
    out.mkdir(parents=True,exist_ok=False);results=[]
    for count in (3,4):
        for shape in ('rectangular','irregular'):
            p=root/f'{count}_{shape}_estimated'
            asset=json.loads((p/'ASSET.json').read_text());records=json.loads((p/'IDENTIFICATION_ROWS.json').read_text())
            offset=np.array(asset['scan_offset']);anchors=np.array(asset['calibrated_anchors'])
            cloud=np.load(p/'scan_world.npy')-offset-[0.,0.,3.]
            h=asset['evaluator'];truth=parameters(h['mass'],np.array(h['com'])-offset,h['inertia'])
            meta=json.loads((p/'PLAN.json').read_text())
            plan=PassagePlan(meta['p0'],meta['R0'],meta['duration'],drop=meta['drop'],formation_mode=meta['formation_mode'])
            regressors=[]
            for t in np.linspace(0.,plan.duration,81):
                target=plan.at(t)
                regressors.append(wrench_matrix(target.rotation,target.omega_world,target.acceleration,target.alpha_world))
            A=np.asarray(regressors)
            for end in (4.,6.,8.,10.,12.,16.):
                prefix=[r for r in records if r['time']<=end+1e-7]
                smoothed=lagged_kinematic_rows(prefix)
                for r in smoothed:
                    for k in ('position','velocity','rotation','omega','uav_position','tension'):r[k]=np.asarray(r[k])
                fit=fit_shape_supported(smoothed,anchors,cloud,window_sec=.4,tension_floor=.01)
                d=decode(fit['theta']);error=np.einsum('tij,j->ti',A,np.asarray(fit['theta'])-truth)
                row=dict(count=count,shape=shape,observation_prefix_sec=end,
                         last_derivative_estimate_time_sec=smoothed[-1]['time'],
                         activated=fit['activated'],rank=fit['rank'],
                         com_error_m=float(np.linalg.norm(d['com']-truth[1:4]/truth[0])),
                         mass_relative_error=float(abs(d['mass']/truth[0]-1.)),
                         future_task_force_rms_error_n=float(np.sqrt(np.mean(np.sum(error[:,:3]**2,axis=1)))),
                         future_task_torque_rms_error_nm=float(np.sqrt(np.mean(np.sum(error[:,3:]**2,axis=1)))))
                results.append(row);save(out/f'{count}_{shape}_{end:g}s_fit.json',fit)
            save(out/'RESULTS.json',results)
    fig,axes=plt.subplots(1,2,figsize=(10,4),layout='constrained')
    for count in (3,4):
        for shape in ('rectangular','irregular'):
            rows=[r for r in results if r['count']==count and r['shape']==shape]
            t=[r['observation_prefix_sec'] for r in rows]
            label=f'{count} UAVs / {shape}'
            axes[0].plot(t,[1000*r['com_error_m'] for r in rows],'-o',ms=3,label=label)
            axes[1].plot(t,[r['future_task_torque_rms_error_nm'] for r in rows],'-o',ms=3,label=label)
    axes[0].set(xlabel='Available observation history (s)',ylabel='3D CoM error (mm)')
    axes[1].set(xlabel='Available observation history (s)',ylabel='Task torque RMS error (N m)')
    for ax in axes:ax.legend(fontsize=8);ax.grid(alpha=.15)
    fig.savefig(out/'causal_identification.png',dpi=180);plt.close(fig)
    save(out/'PROTOCOL.json',dict(source=str(root),kind='Causal prefix reanalysis of four development trajectories',
        smoothing='Nine-frame fixed-lag derivative; records after each prefix cutoff excluded',
        scoring='Evaluator inertia read only after each shape/tension fit; same planned future task for all prefixes',
        independence='No additional flight trials; observation prefixes share the original trajectory'))
    print(results,flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--cohort',required=True);p.add_argument('--out',required=True)
    a=p.parse_args();run(Path(a.cohort).resolve(),Path(a.out).resolve())
