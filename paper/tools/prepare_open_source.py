"""Assemble first-party executable source and public reproducibility records."""
from pathlib import Path
import ast,shutil,json,hashlib,re
ROOT=Path(__file__).resolve().parents[1];LAB=ROOT.parent
REPO=LAB/'github/ForestSling'

def cp(a,b):b.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(a,b)
def main():
 REPO.mkdir(parents=True,exist_ok=True)
 modules={'sixdof_sling','sling_rgbd','inertial_identification','shape_tension_identification',
          'tension_transport','tension_passage','smooth_sling_forces','complex_sling_course','payload_rigging'}
 pending=list(modules)
 while pending:
  name=pending.pop();p=LAB/'forestlab'/f'{name}.py'
  for node in ast.walk(ast.parse(p.read_text(encoding='utf-8'))):
   if isinstance(node,ast.ImportFrom) and node.level==1 and node.module:
    other=node.module.split('.')[0]
    if other not in modules:modules.add(other);pending.append(other)
 for name in sorted(modules):
  cp(LAB/'forestlab'/f'{name}.py',REPO/'forestlab'/f'{name}.py')
 (REPO/'forestlab/__init__.py').write_text('"""Cooperative cable-suspended rigid-payload transport."""\n',encoding='utf-8')
 drivers=['vision_closed_loop.py','review_vision_closed_loop.py','review_batch.py','sixdof_transport.py',
          'sixdof_identification.py','independent_rigid_replay.py','engine_convergence.py','identification_prefixes.py']
 for name in drivers:cp(LAB/'research/vision_tension_20261025'/name,REPO/'research/vision_tension_20261025'/name)
 for name in ['test_inertial_identification.py','test_sixdof_sling.py']:
  cp(LAB/'tests'/name,REPO/'tests'/name)
 # Port the existing identification tests without importing unrelated legacy
 # forest benchmarks merely for their path setup / Euler conversion helper.
 test=REPO/'tests/test_inertial_identification.py'
 t=test.read_text(encoding='utf-8').replace('import run_persistent_flow\n','')
 t=t.replace('from forestlab.inertial_bench import excitation_reference\n','')
 t=t.replace('from forestlab.pose_control import rpy_rotation',
             'from scipy.spatial.transform import Rotation\n\ndef rpy_rotation(degrees):\n    return Rotation.from_euler("xyz",degrees,degrees=True).as_matrix()')
 t=re.sub(r'    def test_excitation_scale_changes_amplitude_without_changing_start\(self\):.*?(?=    def test_)','',t,flags=re.S)
 test.write_text(t,encoding='utf-8')
 # S1 stores exactly the inputs needed for archived result/figure regeneration.
 src=ROOT/'supplementary/S1'
 for p in src.rglob('*'):
  if p.is_file() and p.relative_to(src).parts[0]=='runs':cp(p,REPO/p.relative_to(src))
 for folder in ['manuscript','tools','editorial']:
  for p in (ROOT/folder).rglob('*'):
   rel=p.relative_to(ROOT)
   if p.is_file() and '__pycache__' not in rel.parts and p.suffix not in ('.pyc',) and 'review_20261027' not in rel.parts:
    cp(p,REPO/'paper'/rel)
 for name in ['evidence.json','FIGURE_SOURCES.json','RECOMPUTATION.json']:
  cp(ROOT/'supplementary'/name,REPO/'paper/supplementary'/name)
 for folder in ['bibliography','review_20261027']:
  for p in (ROOT/'supplementary'/folder).rglob('*'):
   if p.is_file():cp(p,REPO/'paper/supplementary'/p.relative_to(ROOT/'supplementary'))
 for p in (LAB/'runs/ast_review_20261027').rglob('*'):
  # Retain full state/command trajectories and identification inputs; exclude
  # redundant 50 Hz pose-prediction JSON and copied code for every new trial.
  if p.is_file() and p.suffix in ('.json','.npz','.npy','.xml','.py','.png'):
   cp(p,REPO/p.relative_to(LAB))
 (REPO/'.gitignore').write_text('__pycache__/\n*.py[cod]\n.venv/\n.deps/\n.pytest_cache/\nartifacts/\nruns/local_*/\n*.log\n',encoding='utf-8')
 (REPO/'requirements.txt').write_text('numpy==2.2.6\nscipy==1.15.3\nmatplotlib==3.10.9\nPillow>=10,<13\nmujoco==3.3.7\nclarabel==0.11.1\nopencv-python>=4.8,<6\npytest>=8,<10\n',encoding='utf-8')
 (REPO/'LICENSE').write_text('''MIT License

Copyright (c) 2026 Fangwei Peng and contributors

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
''',encoding='utf-8')
 (REPO/'README.md').write_text('''# ForestSling

Vision- and tension-informed inertial identification and formation reconfiguration
for cooperative aerial transport through known obstacles.

![Continuous cooperative transport](paper/manuscript/figures/fig01_overview.png)

The model includes three or four free quadrotors, a rigid nonconvex payload,
unilateral compliant cables, first-order motor dynamics, virtual RGB-D pose
registration, integral inertial identification and conic force planning.
The payload reference family and obstacle map are supplied; force allocation,
support endpoint geometry and finite timing/lowering choices are computed.

## Installation

Use Python 3.10 or newer in a clean virtual environment:

```sh
python -m pip install -r requirements.txt
```

The pinned numerical packages match the archived experiment environment. A fresh
Python 3.12 installation also passed all 12 tests and completed an RGB-D
closed-loop transport run. The archived experiments used Python 3.10 and OpenCV
5.0.0; the portability run used OpenCV 4.14.0.94. Wall-clock image arrivals can
vary between executions.

MuJoCo image capture requires a working OpenGL context. Numerical analysis and
archived figure regeneration do not require Unreal Engine.

## Recompute the archived results

```sh
python paper/tools/recompute_results.py
python paper/tools/build_figures.py
python paper/tools/review_analysis.py
python paper/tools/review_replay.py
python paper/tools/review_aggregate.py
python -m pytest tests -q
```

`runs/` contains original dynamic records, sensor records needed by the
identification analysis and the revision experiments. Every task has a
protocol and evaluator mass properties. `paper/supplementary/review_20261027/`
contains the complete inertial audit, held-out prediction, declared
observation-space sensitivity and independent replay results. Reanalysis of
archived records is distinct from new closed-loop executions.

## Run a new image-based closed-loop experiment

```sh
python research/vision_tension_20261025/vision_closed_loop.py --out runs/local_four_irregular --beam --count 4 --shape irregular --method estimated --duration 20 --physics-dt .0005
```

For the large-angle forked course, add `--course --corner-mounts --pose-mode
large --shape forked --duration 125`. Each output directory must be new.
The controller runs at 50 Hz and the virtual RGB-D worker requests images at
10 Hz. Initial whole-segment planning runs while the system remains in hover.

The frozen five-arm ablation and 20 independent sensor-seed repetitions are
specified by `research/vision_tension_20261025/review_batch.py`. That script
retains failed attempts; it does not silently drop unsuccessful trials.

For wall-clock experiments, run the simulator without concurrent heavy analysis.
Image-age gate failures and explicitly labeled retries remain separate records.
The original first-attempt seed cohort is never replaced by retry outcomes.

## Source and visual assets

First-party code is licensed under MIT. NumPy, SciPy, Matplotlib, Pillow,
OpenCV, Clarabel and MuJoCo remain subject to their respective upstream
licenses and are installed as dependencies rather than vendored here.
The original Unreal CityPark project and proprietary scene/vehicle assets
are excluded. Camera figures are rendered outputs of recorded dynamic
states, not a second physical execution. The numerical model and its RGB-D
sensor do not depend on CityPark assets.

## Citation

The associated manuscript targets Aerospace Science and Technology. No
publication DOI is assigned to this repository snapshot. Author and funding
information appears in `paper/manuscript/author_information.tex`.
''',encoding='utf-8')
 (REPO/'CITATION.cff').write_text('''cff-version: 1.2.0
message: "Please cite this software and the associated manuscript."
title: "ForestSling: Cooperative aerial transport with visual and tension sensing"
type: software
authors:
  - family-names: Peng
    given-names: Fangwei
  - family-names: Qi
    given-names: Juntong
  - family-names: Peng
    given-names: Yan
  - family-names: Wang
    given-names: Mingming
  - family-names: Wu
    given-names: Chong
repository-code: "https://github.com/shepherd-p/ForestSling"
license: MIT
version: "0.1.0"
date-released: "2026-10-27"
''',encoding='utf-8')
 files=[p for p in REPO.rglob('*') if p.is_file() and '.git' not in p.parts and '__pycache__' not in p.parts]
 large=[str(p.relative_to(REPO)) for p in files if p.stat().st_size>95_000_000]
 assert not large,large
 print(json.dumps(dict(repository=str(REPO),files=len(files),size_MB=sum(p.stat().st_size for p in files)/1e6,modules=len(modules)),indent=2))

if __name__=='__main__':main()
