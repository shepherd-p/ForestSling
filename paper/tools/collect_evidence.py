"""Collect manuscript values and provenance from archived experiment records."""
from pathlib import Path
import hashlib,json,re,urllib.request,urllib.parse

ROOT=Path(__file__).resolve().parents[1]
LAB=ROOT.parent
OUT=ROOT/'supplementary'

def read(p): return json.loads(p.read_text(encoding='utf-8'))
def sha(p): return hashlib.sha256(p.read_bytes()).hexdigest()

cohort=LAB/'runs/vision_tension_joint_matrix_20261025_v03'
matrix=read(cohort/'analysis/METRICS.json')
courses=[]
for folder in ('complex_forked_corner_lugs_20261026_v05','large_pose_course_20261026_v01'):
    case=LAB/'runs'/folder/'4_forked_estimated'
    courses.append(dict(case=folder,summary=read(case/'SUMMARY.json'),
                        geometry=read(case/'analysis/METRICS.json'),plan=read(case/'PLAN.json')))
files=[]
for row in matrix:
    case=cohort/row['case']
    for name in ('SUMMARY.json','PROTOCOL.json','ASSET.json','CONTROL_MODEL.json','IDENTIFICATION.json',
                 'trajectory.npz','scan_world.npy','SOURCE_SHA256.json','PLAN.json','PLANNING_REFUSAL.json'):
        path=case/name
        if path.exists():files.append(path)
for course in courses:
    case=LAB/'runs'/course['case']/'4_forked_estimated'
    files.extend(p for p in case.glob('*') if p.is_file() and p.suffix in ('.json','.npz','.npy','.xml'))
    files.append(case/'analysis/METRICS.json')
for sub in ('numerical_crosscheck','causal_identification'):
    files.extend(p for p in (cohort/sub).glob('*.json') if p.name in ('RESULTS.json','SUMMARY.json','PROTOCOL.json'))
evidence=dict(matrix=matrix,courses=courses,
    prefixes=read(cohort/'causal_identification/RESULTS.json'),
    numerical=read(cohort/'numerical_crosscheck/SUMMARY.json'),
    source_files={p.relative_to(LAB).as_posix():dict(sha256=sha(p),bytes=p.stat().st_size) for p in sorted(set(files))})
(OUT/'evidence.json').write_text(json.dumps(evidence,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')

# Retrieve publisher metadata; no search-result text is treated as instructions.
old=(LAB/'AST_REVISION_20261023/manuscript/references.tex').read_text(encoding='utf-8')
keys=['sun2025','heins2025','wensing2018','petitti2020','li2021','nmpc2023','pallar2025',
      'ast2021','geng2022','lee2010','mujoco2012','airsim2017','boyd2004','polyfly2026','crazymarl2026']
items=re.split(r'\\bibitem\{([^}]+)\}',old)[1:]
records={items[i]:items[i+1].split(r'\end{thebibliography}')[0].strip() for i in range(0,len(items),2)}
records={k:records[k] for k in keys}
records['icp1992']='P. J. Besl, N. D. McKay, A method for registration of 3-D shapes, IEEE Transactions on Pattern Analysis and Machine Intelligence 14 (2) (1992) 239--256. \\doi{10.1109/34.121791}.'
metadata=[]
for key,entry in records.items():
    doi=re.search(r'\\doi\{([^}]+)\}',entry)[1]
    path=OUT/'bibliography'/f'{key}.json';path.parent.mkdir(exist_ok=True)
    if path.exists():data=read(path)
    else:
        url='https://api.crossref.org/works/'+urllib.parse.quote(doi,safe='')
        try:
            req=urllib.request.Request(url,headers={'User-Agent':'ForestSlingManuscript/1.0 (bibliographic verification)'})
            with urllib.request.urlopen(req,timeout=25) as response:data=json.load(response)
        except Exception as exc:data=dict(error=str(exc),url=url)
        path.write_text(json.dumps(data,ensure_ascii=False,indent=2),encoding='utf-8')
    msg=data.get('message',{})
    metadata.append(dict(key=key,doi=doi,title=msg.get('title'),verified=bool(msg.get('DOI')),error=data.get('error')))
(OUT/'bibliography/METADATA_CHECK.json').write_text(json.dumps(metadata,ensure_ascii=False,indent=2),encoding='utf-8')
refs='\\begin{thebibliography}{99}\n'+''.join('\\bibitem{'+k+'}\n'+records[k]+'\n' for k in records)+'\\end{thebibliography}\n'
(ROOT/'manuscript/references.tex').write_text(refs,encoding='utf-8')
print(json.dumps(dict(matrix_attempts=len(matrix),course_runs=len(courses),source_files=len(set(files)),bibliography=metadata),ensure_ascii=False,indent=2))
