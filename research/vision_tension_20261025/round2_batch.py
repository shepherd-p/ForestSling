"""Second-round initial attempts; no retry replaces an earlier observation."""
from pathlib import Path
import subprocess,sys,json,time
ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'runs/ast_round2_20261027'
def main():
 OUT.mkdir(exist_ok=True)
 jobs=[dict(name=f'4_rectangular_estimated_{seed}',seed=seed,shape='rectangular',method='estimated',perturbation='none',course=False) for seed in range(27000,27020)]
 jobs += [dict(name=f'4_{shape}_unconstrained_28000',seed=28000,shape=shape,method='unconstrained',perturbation='none',course=shape=='forked') for shape in ('rectangular','forked')]
 jobs += [dict(name=f'4_{shape}_moment_psd_28000',seed=28000,shape=shape,method='moment_psd',perturbation='none',course=shape=='forked') for shape in ('rectangular','forked')]
 jobs += [dict(name=f'4_{shape}_{mode}_28001',seed=28001,shape=shape,method='estimated',perturbation=mode,course=shape=='forked') for shape in ('rectangular','forked') for mode in ('tension_bias','motor_tau','observation_loss')]
 protocol=dict(jobs=jobs,age_gate_sec=.35,cohort='Same twenty seeds as original, one new initial attempt each, process-isolated image worker',perturbations=dict(tension_bias_N=[.04,-.04,.04,-.04],motor_time_constant_sec=.0375,observation_loss='Discard every tenth delivered observation; no ground-truth pose replacement'),scope='Two configurations, three declared perturbations, one execution per condition; not a population robustness estimate')
 (OUT/'PROTOCOL.json').write_text(json.dumps(protocol,indent=2),encoding='utf-8')
 results=[]
 for j in jobs:
  folder=OUT/j['name']
  if not (folder/'SUMMARY.json').exists():
   if folder.exists():raise RuntimeError('Incomplete attempt '+j['name'])
   cmd=[sys.executable,str(Path(__file__).with_name('round2_vision_closed_loop.py')),'--out',str(folder),'--count','4','--beam','--shape',j['shape'],'--method',j['method'],'--sensor-seed',str(j['seed']),'--duration','125' if j['course'] else '20','--perturbation',j['perturbation']]
   if j['course']:cmd+=['--course','--corner-mounts','--pose-mode','large']
   print('START '+j['name'],flush=True)
   with (OUT/(j['name']+'.log')).open('w') as f:r=subprocess.run(cmd,cwd=ROOT,stdout=f,stderr=subprocess.STDOUT)
   if r.returncode:raise RuntimeError(f'Child failed {j["name"]}: {r.returncode}')
  s=json.loads((folder/'SUMMARY.json').read_text(encoding='utf-8'));results.append(dict(case=j['name'],**s))
  (OUT/'RESULTS.json').write_text(json.dumps(results,indent=2),encoding='utf-8')
  print(json.dumps(dict(case=j['name'],passed=s['passed'],reason=s['reason'])),flush=True)
 print('ALL SECOND-ROUND ATTEMPTS COMPLETE',flush=True)
if __name__=='__main__':main()
