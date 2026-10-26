"""Preserve every revision attempt and separate acquisition, planning and execution."""
from pathlib import Path
import json,csv,math
import numpy as np
ROOT=Path(__file__).resolve().parents[1];LAB=ROOT if (ROOT/'runs').is_dir() else ROOT.parent
RUN=LAB/'runs/ast_review_20261027';OUT=ROOT/'supplementary/review_20261027'

def read(p):return json.loads(p.read_text(encoding='utf-8'))
def write(p,obj):p.write_text(json.dumps(obj,indent=2,allow_nan=False)+'\n',encoding='utf-8')
def interval(k,n):
    z=1.95996398454;den=1+z*z/n;center=(k/n+z*z/(2*n))/den
    half=z*math.sqrt(k/n*(1-k/n)/n+z*z/(4*n*n))/den
    return [max(0.,center-half),min(1.,center+half)]
def main():
    rows=[]
    for p in sorted(RUN.glob('*/SUMMARY.json')):
        s=read(p);name=p.parent.name;seed=int(name.split('_')[-1]) if '_retry' not in name else int(name.split('_retry')[0].split('_')[-1])
        row=dict(case=name,seed=seed,retry='_retry' in name,count=s['count'],shape=s['shape'],method=s['method'],
            planned=s['motion_start_sec'] is not None,completed=s['completed'],passed=s['passed'],reason=s['reason'],
            peak_position_cm=s['max_position_error_m']*100 if s['max_position_error_m'] is not None else None,peak_attitude_deg=s['max_attitude_error_deg'],
            min_tension_N=s['min_tension_n'],max_tension_N=s['max_tension_n'],
            planning_sec=s['initial_planning_wall_sec'],com_error_mm=s.get('com_error_m')*1000 if s.get('com_error_m') is not None else None,
            contacts=s['contacts'],cable_contact_steps=s['cable_contact_steps'])
        d=np.load(p.parent/'trajectory.npz');errors=d['errors']
        # Full-transport distributions exclude aborted partial trajectories.
        if s['completed'] and len(errors):
            errors=errors[errors[:,0]<=s['motion_start_sec']+s['planned_motion_duration_sec']+1e-7]
            row['position_rms_cm']=float(np.sqrt(np.mean(errors[:,1]**2))*100)
            row['attitude_rms_deg']=float(np.sqrt(np.mean(errors[:,2]**2)))
            states=d['states'];n=s['count'];q=states[:,1:1+7*(n+1)].reshape(-1,n+1,7)
            distances=[np.linalg.norm(q[:,i,:3]-q[:,j,:3],axis=1).min() for i in range(1,n+1) for j in range(1,i)]
            row['minimum_aircraft_center_distance_m']=float(min(distances))
        rows.append(row)
    methods=['mass_matched','mass_only','mass_com','estimated','oracle']
    first=[r for r in rows if r['seed']==2501 and not r['retry']]
    assert len(first)==20
    repeats=[r for r in rows if 27000<=r['seed']<27020 and not r['retry']]
    assert len(repeats)==20
    completed=[r for r in repeats if r['completed']];success=sum(r['passed'] for r in repeats)
    distributions={}
    for field in ['peak_position_cm','peak_attitude_deg','position_rms_cm','attitude_rms_deg','com_error_mm','planning_sec','minimum_aircraft_center_distance_m']:
        values=np.array([r[field] for r in completed]);distributions[field]=dict(min=float(values.min()),median=float(np.median(values)),p95=float(np.quantile(values,.95)),max=float(values.max()))
    stats=dict(first_attempts=20,passed=success,completed=len(completed),planning_accepted=sum(r['planned'] for r in repeats),
        acquisition_aborts=sum(r['reason']=='image_tracking_stale' for r in repeats),wilson95_all_attempts=interval(success,20),
        conditional_success_among_completed=sum(r['passed'] for r in completed),wilson95_completed=interval(sum(r['passed'] for r in completed),len(completed)),
        completed_transport_distributions=distributions,
        scope='One fixed four-aircraft rectangular known-beam configuration with independent sensor seeds; fixed hidden ballast',
        retry_policy='Initial attempts remain the statistical cohort. Explicitly labeled diagnostic retries are retained separately and never replace the original outcomes.')
    write(OUT/'TRIAL_METRICS.json',rows);write(OUT/'REPEATED_SEED_STATISTICS.json',stats)
    with (OUT/'TRIAL_METRICS.csv').open('w',encoding='utf-8',newline='') as f:
        fields=list(dict.fromkeys(k for r in rows for k in r));w=csv.DictWriter(f,fieldnames=fields);w.writeheader();w.writerows(rows)
    lines=[]
    for n in (3,4):
        for shape in ('rectangular','irregular'):
            cells=[]
            for method in methods:
                r=next(r for r in first if r['count']==n and r['shape']==shape and r['method']==method)
                value=f"{r['peak_attitude_deg']:.2f}"+('\\textsuperscript{*}' if not r['passed'] else '') if r['completed'] else ('A' if r['reason']=='image_tracking_stale' else 'R')
                cells.append(value)
            lines.append(f"{n} & {shape.capitalize()} & "+' & '.join(cells)+r'\\')
    (ROOT/'manuscript/table_ablation_rows.tex').write_text('\n'.join(lines).rstrip('\\')+'\n',encoding='utf-8')
    print(json.dumps(stats,indent=2))
if __name__=='__main__':main()
