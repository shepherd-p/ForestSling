"""Frozen paired mass/CoM/inertia ablation and independent nominal sensor seeds."""
from pathlib import Path
import json,subprocess,sys,time
ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'runs/ast_review_20261027'

def main():
    OUT.mkdir(exist_ok=True)
    jobs=[(n,s,m,2501) for n in (3,4) for s in ('rectangular','irregular')
          for m in ('mass_matched','mass_only','mass_com','estimated','oracle')]
    jobs += [(4,'rectangular','estimated',27000+i) for i in range(20)]
    (OUT/'PROTOCOL.json').write_text(json.dumps(dict(
        task='8 m known-beam transport, same duration/lowering candidates and control gains',
        ablation_methods=['mass_matched','mass_only','mass_com','estimated','oracle'],
        repeats='20 new independent depth/tension/telemetry seeds; fixed geometry and ballast',
        seeds=list(range(27000,27020)),physics_step_sec=.0005,jobs=jobs),indent=2))
    rows=[]
    for n,s,m,seed in jobs:
        name=f'{n}_{s}_{m}_{seed}';folder=OUT/name
        if not (folder/'SUMMARY.json').exists():
            if folder.exists():raise RuntimeError(f'Incomplete existing job: {folder}')
            print('START '+name,flush=True)
            cmd=[sys.executable,str(Path(__file__).with_name('review_vision_closed_loop.py')),
                 '--out',str(folder),'--beam','--count',str(n),'--shape',s,'--method',m,
                 '--duration','20','--physics-dt','.0005','--sensor-seed',str(seed)]
            with (OUT/(name+'.log')).open('w') as log:
                code=subprocess.run(cmd,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT).returncode
            if code or not (folder/'SUMMARY.json').exists():raise RuntimeError(f'Job failed: {name}, code {code}')
        row=json.loads((folder/'SUMMARY.json').read_text());row.update(case=name,seed=seed)
        rows.append(row);(OUT/'RESULTS.json').write_text(json.dumps(rows,indent=2))
        print(json.dumps({k:row[k] for k in ('case','passed','reason','max_attitude_error_deg','initial_planning_wall_sec')}),flush=True)
    print('ALL JOBS COMPLETE',flush=True)

if __name__=='__main__':main()
