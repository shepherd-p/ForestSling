"""Aggregate completed second-round cohorts and exposure-to-control timelines."""
from pathlib import Path
import json,sys,math,csv
import numpy as np
ROOT=Path(__file__).resolve().parents[1];LAB=ROOT if (ROOT/'runs').is_dir() else ROOT.parent
sys.path[:0]=[str(LAB),str(LAB/'.deps/markerless')]
from forestlab.tension_passage import box_sdf
from review_aggregate import interval
OUT=ROOT/'supplementary/round2_20261027';RUN=LAB/'runs/ast_round2_20261027'
def read(p):return json.loads(p.read_text(encoding='utf-8'))
def dist(v):
 a=np.asarray(v);return dict(min=float(a.min()),median=float(np.median(a)),p95=float(np.quantile(a,.95)),max=float(a.max()))

def identities():
 rows=[]
 folders=[('N',LAB/'runs/vision_tension_joint_matrix_20261025_v03'),('A',LAB/'runs/ast_review_20261027'),('B',RUN)]
 evidence=read(ROOT/'supplementary/evidence.json')
 folders += [('N-course',LAB/'runs'/r['case']) for r in evidence['courses']]
 for group,folder in folders:
  for p in sorted(folder.glob('*/SUMMARY.json')):
   if not (p.parent/'PROTOCOL.json').exists():continue
   s=read(p);pr=read(p.parent/'PROTOCOL.json');manifest=p.parent/'SOURCE_SHA256.json'
   assert manifest.exists(),str(manifest)
   rows.append(dict(group=group,experiment_id=p.parent.relative_to(LAB/'runs').as_posix(),seed=pr.get('seed'),
     ballast_seed=pr.get('ballast_seed',2501),count=s['count'],shape=s['shape'],method=s['method'],
     perturbation=pr.get('perturbation','none'),diagnostic_retry='_retry' in p.parent.name,
     planned=s.get('motion_start_sec') is not None,completed=s['completed'],passed=s['passed'],reason=s['reason'],
     peak_attitude_deg=s.get('max_attitude_error_deg'),trajectory_available=(p.parent/'trajectory.npz').exists(),
     source_hash_manifest=manifest.relative_to(LAB).as_posix()))
 (OUT/'EXPERIMENT_IDENTITIES.json').write_text(json.dumps(rows,indent=2)+'\n',encoding='utf-8')
 with (OUT/'EXPERIMENT_IDENTITIES.csv').open('w',encoding='utf-8',newline='') as f:
  w=csv.DictWriter(f,fieldnames=rows[0].keys());w.writeheader();w.writerows(rows)
def aggregate():
 repeats=[];allrows=[];delays={k:[] for k in ('queue_sec','processing_sec','return_and_read_sec','exposure_to_read_sec','received_sensor_age_sec')}
 for p in sorted(RUN.glob('*/SUMMARY.json')):
  s=read(p);protocol=read(p.parent/'PROTOCOL.json');row=dict(case=p.parent.name,seed=protocol['seed'],perturbation=protocol['perturbation'],**s)
  if not (p.parent/'trajectory.npz').exists():
   allrows.append(row);continue
  raw=np.load(p.parent/'trajectory.npz');err=raw['errors'];state=raw['states'];q=state[:,1:36].reshape(-1,5,7)
  row['minimum_aircraft_center_distance_m']=min(float(np.linalg.norm(q[:,i,:3]-q[:,j,:3],axis=1).min()) for i in range(1,5) for j in range(1,i))
  row['minimum_aircraft_clearance_m']=min(float(box_sdf(q[:,1:,:3],b,.28).min()) for b in protocol['obstacles'])
  if s['completed'] and len(err):
   er=err[err[:,0]<=s['motion_start_sec']+s['planned_motion_duration_sec']+1e-6]
   row.update(position_rms_cm=float(np.sqrt(np.mean(er[:,1]**2))*100),attitude_rms_deg=float(np.sqrt(np.mean(er[:,2]**2))))
  if 27000<=protocol['seed']<27020 and protocol['perturbation']=='none' and s['method']=='estimated':
   repeats.append(row)
   for v in read(p.parent/'VISION.json'):
    for key in delays:delays[key].append(v[key])
  allrows.append(row)
 assert len(repeats)==20,len(repeats)
 complete=[r for r in repeats if r['completed']];success=sum(r['passed'] for r in repeats)
 stats=dict(first_attempts=20,completed=len(complete),passed=success,wilson95_all_attempts=interval(success,20),
  gate_sec=.35,source='Process-isolated sensor driver; same seeds 27000--27019, fixed configuration and ballast',
  delay_distributions_ms={k:dist(np.array(v)*1000) for k,v in delays.items()},
  completed_transport_distributions={k:dist([r[k] for r in complete]) for k in ('position_rms_cm','attitude_rms_deg','max_attitude_error_deg','mass_relative_error','minimum_aircraft_clearance_m','minimum_aircraft_center_distance_m')},
  aborts=[dict(case=r['case'],reason=r['reason']) for r in repeats if not r['completed']],
  independence='New initial attempts with the same sensor seeds; historical attempts retained. Anonymous-clone tests are author-run portability checks, not third-party replication.')
 for name,value in [('REPEATED_SEED_STATISTICS.json',stats),('TRIAL_METRICS.json',allrows)]:
  (OUT/name).write_text(json.dumps(value,indent=2)+'\n',encoding='utf-8')
 identities()
 print(json.dumps(dict(passed=success,attempts=len(repeats),all_completed_runs=len(allrows)),indent=2))
 return stats,allrows
if __name__=='__main__':aggregate()
