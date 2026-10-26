"""Assemble portable numerical evidence and original task videos."""
from pathlib import Path
import hashlib,json,shutil,html
ROOT=Path(__file__).resolve().parents[1];LAB=ROOT.parent
SUP=ROOT/'supplementary';S1=SUP/'S1';S1.mkdir(exist_ok=True)
E=json.loads((SUP/'evidence.json').read_text(encoding='utf-8'))

def copy(source,destination):
    destination.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(source,destination)

for relative in E['source_files']:copy(LAB/relative,S1/relative)
bindings=json.loads((SUP/'FIGURE_SOURCES.json').read_text(encoding='utf-8'))
for item in bindings:copy(LAB/item['file'],S1/item['file'])
base=LAB/'runs/vision_tension_joint_matrix_20261025_v03'
for row in E['matrix']:
    folder=base/row['case']
    for name in ('IDENTIFICATION_ROWS.json','SMOOTHED_IDENTIFICATION_ROWS.json','SENSOR_ROWS.json','VISION.json','model.xml'):
        p=folder/name
        if p.exists():copy(p,S1/p.relative_to(LAB))
folders=[base/r['case'] for r in E['matrix']]+[LAB/'runs'/r['case']/'4_forked_estimated' for r in E['courses']]
for folder in folders:
    for p in (folder/'source_snapshot').rglob('*.py'):copy(p,S1/p.relative_to(LAB))
fine=LAB/'runs/vision_tension_beam_20261025_refined_dt_v01'
for p in fine.iterdir():
    if p.is_file() and p.suffix in ('.json','.npz','.npy','.xml'):copy(p,S1/p.relative_to(LAB))
for sub in ('numerical_crosscheck','causal_identification'):
    for p in (base/sub).iterdir():
        if p.is_file() and p.suffix in ('.json','.npz'):copy(p,S1/p.relative_to(LAB))
for name in ('evidence.json','FIGURE_SOURCES.json','RECOMPUTATION.json'):copy(SUP/name,S1/'supplementary'/name)
for name in ('build_figures.py','paper_diagrams.py','review_figures.py','recompute_results.py','review_aggregate.py'):
    copy(ROOT/'tools'/name,S1/'tools'/name)
for p in (SUP/'review_20261027').rglob('*'):
    if p.is_file():copy(p,S1/'supplementary'/p.relative_to(SUP))
for p in (LAB/'runs/ast_review_20261027').rglob('*'):
    if p.is_file() and p.suffix in ('.json','.npz','.npy','.xml','.py','.png'):
        copy(p,S1/p.relative_to(LAB))
for name in ('independent_rigid_replay.py','engine_convergence.py','identification_prefixes.py'):
    copy(LAB/'research/vision_tension_20261025'/name,S1/'verification_source'/name)
(S1/'requirements-analysis.txt').write_text('numpy>=1.24\nscipy>=1.10\nmatplotlib>=3.7\nPillow>=10\n',encoding='utf-8')
(S1/'README.md').write_text('''# Numerical supplement

The archive contains eight paired base task attempts, two forked-payload course executions, a refined-step base execution, causal observation-prefix fits and independent command replay. The revision adds a five-arm controlled ablation (20 initial attempts), 20 independent sensor-seed attempts, one explicitly labeled diagnostic acquisition retry, 160 observation-space sensitivity fits and three demanding replay intervals. Original source snapshots are stored beside the corresponding run records. All quantities are numerical simulation data. Executable first-party code and rerun instructions are also public at https://github.com/shepherd-p/ForestSling.

## Recompute the manuscript values

Use Python 3.10 or newer with NumPy. From the extracted directory, run:

    python tools/recompute_results.py

This checks the archived-input hashes and recomputes position/attitude error extrema, control-model CoM errors and course aircraft-center separation. Tension extrema in the source summaries include physical substeps; the script verifies that saved sampled tensions lie inside those extrema.

## Regenerate figures

Install requirements-analysis.txt, then run:

    python tools/build_figures.py

    python tools/review_aggregate.py

The figures and table rows are written to manuscript/. Original Unreal RGB inputs are retained at their source-relative paths. Numerical curves use trajectory.npz and the stored experiment summaries. No simulation is run by these two analysis commands.

## Dynamics and numerical verification

The run-specific source_snapshot folders retain the simulation, sensing, identification and planning code used for those runs. The engine uses MuJoCo, SciPy and Clarabel in addition to NumPy and Pillow; rendering requires an OpenGL-capable environment. The independent motor-input replay implementation and its original driver are in verification_source/. Its reported comparison interval is 29.30--31.30 s of the three-aircraft irregular base execution, not a second rendering of the full course.

The virtual RGB-D configuration, fixed preparation period, known obstacle maps and candidate motion families are recorded in each PROTOCOL.json. Evaluator mass properties in ASSET.json are ground truth for metric calculation and dynamics, not inputs to the sensing estimator. The fitted support weights represent inertial moments, not a unique density reconstruction.

MANIFEST_SHA256.json identifies the archived files. The video files and publisher-facing captions are supplied separately. Proprietary Unreal scene assets are not included.
''',encoding='utf-8')
videos=[
 ('S2a_three_UAVs_rectangular.mp4','Three UAVs / rectangular','unreal_vision_tension_20261026_v02/3_rectangular_estimated'),
 ('S2b_three_UAVs_irregular.mp4','Three UAVs / irregular','unreal_vision_tension_20261026_v02/3_irregular_estimated'),
 ('S2c_four_UAVs_rectangular.mp4','Four UAVs / rectangular','unreal_vision_tension_20261026_v02/4_rectangular_estimated'),
 ('S2d_four_UAVs_irregular.mp4','Four UAVs / irregular','unreal_vision_tension_20261026_v02/4_irregular_estimated'),
 ('S3_forked_sequential_apertures.mp4','Forked load / sequential apertures','unreal_corner_rigging_20261026_v01/4_forked_estimated'),
 ('S4_forked_large_reorientation.mp4','Forked load / large reorientation','unreal_large_pose_20261026_v01/4_forked_estimated')]
items=[]
for name,title,relative in videos:
    source=LAB/'runs'/relative/'unreal_original.mp4';destination=SUP/'videos'/name
    if not destination.exists() or destination.stat().st_size!=source.stat().st_size:copy(source,destination)
    assert hashlib.sha256(source.read_bytes()).digest()==hashlib.sha256(destination.read_bytes()).digest()
    items.append(dict(file='videos/'+name,title=title,source=source.relative_to(LAB).as_posix(),sha256=hashlib.sha256(destination.read_bytes()).hexdigest(),bytes=destination.stat().st_size))
    audit=LAB/'runs'/relative.split('/')[0]/'AUDIT.json'
    if audit.exists():copy(audit,SUP/'video_audits'/relative.split('/')[0]/'AUDIT.json')
(SUP/'VIDEO_MANIFEST.json').write_text(json.dumps(items,ensure_ascii=False,indent=2),encoding='utf-8')
cards=''.join('<section><h2>'+html.escape(i['title'])+'</h2><video controls preload="metadata" src="'+i['file']+'"></video></section>' for i in items)
(SUP/'video_overview.html').write_text('<!doctype html><html lang="en"><meta charset="utf-8"><title>Cooperative aerial transport videos</title><style>body{font:16px/1.6 Arial;background:#edf2f4;color:#203a49;max-width:1200px;margin:24px auto;padding:0 18px}section{background:white;padding:18px;margin:20px 0;border-radius:8px}h2{font-size:20px}video{display:block;width:100%;background:#18232b}a{color:#176d91}</style><h1>Cooperative aerial transport</h1><p>Recorded motor and cable closed-loop trajectories rendered in Unreal Engine. 1080p / 25 fps / 1× physical time.</p><p><a href="../ForestSling_AST_Manuscript.pdf">Manuscript</a> · <a href="../editorial/supplementary_captions.txt">Video captions</a></p>'+cards+'</html>',encoding='utf-8')
print(json.dumps(dict(numerical_files=sum(p.is_file() for p in S1.rglob('*')),videos=len(items),video_MB=sum(i['bytes'] for i in items)/1e6),indent=2))
