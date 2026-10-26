"""Package the reviewed manuscript, portable numerical supplement and videos."""
from pathlib import Path
import hashlib
import json
import re
import zipfile

ROOT = Path(__file__).resolve().parents[1]
S1 = ROOT / 'supplementary/S1'


def digest(path):
    h = hashlib.sha256()
    with path.open('rb') as f:
        for block in iter(lambda: f.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def record(path, base):
    return dict(file=path.relative_to(base).as_posix(), bytes=path.stat().st_size,
                sha256=digest(path))


def write_json(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')


def archive(path, members, extra=None):
    with zipfile.ZipFile(path, 'w', compression=zipfile.ZIP_DEFLATED, compresslevel=6) as z:
        names = []
        for source, name in members:
            name = str(name).replace('\\', '/')
            assert name not in names and not name.startswith('/') and '..' not in Path(name).parts
            names.append(name)
            compression = zipfile.ZIP_STORED if source.suffix.lower() == '.mp4' else zipfile.ZIP_DEFLATED
            z.write(source, name, compress_type=compression)
        for name, value in (extra or {}).items():
            assert name not in names
            z.writestr(name, value)
    with zipfile.ZipFile(path) as z:
        assert len(z.namelist()) == len(set(z.namelist()))
        assert z.testzip() is None
    return record(path, ROOT)


def main():
    quality = json.loads((ROOT / 'qa/QUALITY_REPORT.json').read_text(encoding='utf-8'))
    assert quality['status'] == 'passed_with_visual_review'
    recomputed = json.loads((ROOT / 'supplementary/RECOMPUTATION.json').read_text(encoding='utf-8'))
    assert recomputed['passed'] and recomputed['source_files_checked'] == 111
    assert json.loads((ROOT / 'qa/REPRODUCIBILITY.json').read_text(encoding='utf-8'))['passed']

    numerical = []
    for p in sorted(S1.rglob('*')):
        if not p.is_file():
            continue
        rel = p.relative_to(S1)
        if rel.parts[0] == 'manuscript' or '__pycache__' in rel.parts or p.suffix == '.pyc':
            continue
        if rel.as_posix() == 'MANIFEST_SHA256.json':
            continue
        numerical.append(p)
    write_json(S1 / 'MANIFEST_SHA256.json', [record(p, S1) for p in numerical])
    numerical.append(S1 / 'MANIFEST_SHA256.json')
    s1zip = ROOT / 'supplementary/S1_numerical_data.zip'
    s1result = archive(s1zip, [(p, p.relative_to(S1).as_posix()) for p in numerical])

    manuscript = ROOT / 'manuscript'
    texfiles = sorted(manuscript.glob('*.tex'))
    pdffigures = sorted((manuscript / 'figures').glob('*.pdf'))
    assert len(pdffigures) == 9
    combined = '\n'.join(p.read_text(encoding='utf-8') for p in texfiles)
    for name in re.findall(r'\\input\{([^}]+)\}', combined):
        assert (manuscript / name).is_file(), name
    for name in re.findall(r'\\includegraphics(?:\[[^\]]*\])?\{([^}]+)\}', combined):
        assert (manuscript / 'figures' / name).is_file(), name
    instructions = (
        'Compile main.tex using XeLaTeX or Tectonic with elsarticle and fontspec.\n'
        'Install Arial and Times New Roman; figure fonts are embedded in the PDF inputs.\n'
        'Required packages are declared in main.tex; all nine figure PDFs are included.\n'
        'Run xelatex main.tex twice, or use tectonic main.tex.\n'
        'References are contained in references.tex; no BibTeX run is required.\n'
        'The companion PDF is the reviewed double-column manuscript.\n'
    )
    (manuscript / 'README_compile.txt').write_text(instructions, encoding='utf-8')
    latexzip = ROOT / 'ForestSling_AST_LaTeX.zip'
    svgfigures = sorted((manuscript / 'figures').glob('*.svg'))
    latexpaths = texfiles + pdffigures + svgfigures + [manuscript / 'README_compile.txt']
    latexresult = archive(latexzip, [(p, p.relative_to(manuscript).as_posix()) for p in latexpaths])

    members = [ROOT / name for name in (
        'ForestSling_AST_Manuscript.pdf', 'ForestSling_AST_Chinese_Manuscript.docx',
        'ForestSling_AST_Chinese_Manuscript.pdf', 'ForestSling_AST_LaTeX.zip',
        'README_阅读与文件说明.md', 'supplementary/S1_numerical_data.zip',
        'supplementary/video_overview.html', 'supplementary/VIDEO_MANIFEST.json',
        'supplementary/FIGURE_SOURCES.json', 'supplementary/evidence.json',
        'supplementary/RECOMPUTATION.json', 'qa/QUALITY_REPORT.json',
        'qa/REPRODUCIBILITY.json', 'manuscript/article_zh.md',
        'tools/build_chinese_manuscript.py', 'tools/package_article.py')]
    members += sorted((ROOT / 'editorial').glob('*.txt'))
    members += sorted(p for p in (ROOT/'editorial/review_20261027').glob('*')
                      if p.is_file() and p.suffix in ('.docx','.md','.json'))
    members += sorted(p for p in (ROOT/'supplementary/review_20261027').rglob('*') if p.is_file())
    members += sorted((ROOT / 'supplementary/videos').glob('*.mp4'))
    members += sorted(p for p in (ROOT / 'supplementary/video_audits').rglob('*') if p.is_file())
    members += sorted(p for p in (ROOT / 'supplementary/bibliography').rglob('*') if p.is_file())
    assert len(list((ROOT / 'supplementary/videos').glob('*.mp4'))) == 6
    for row in json.loads((ROOT / 'supplementary/VIDEO_MANIFEST.json').read_text(encoding='utf-8')):
        p = ROOT / 'supplementary' / row['file']
        assert digest(p) == row['sha256'] and p.stat().st_size == row['bytes']
    html = (ROOT / 'supplementary/video_overview.html').read_text(encoding='utf-8')
    for url in re.findall(r'(?:src|href)=[\"\']([^\"\']+)[\"\']', html):
        if not url.startswith(('http:', 'https:', '#')):
            assert (ROOT / 'supplementary' / url).exists(), url
    content = [record(p, ROOT) for p in members]
    bundle = ROOT / 'ForestSling_AST_Article_Package.zip'
    result = archive(bundle, [(p, p.relative_to(ROOT).as_posix()) for p in members],
                     {'PACKAGE_MANIFEST_SHA256.json': json.dumps(content, ensure_ascii=False, indent=2)})
    manifest = dict(status='passed', components=content, archives=[s1result, latexresult, result])
    write_json(ROOT / 'SHA256_MANIFEST.json', manifest)
    print(json.dumps(dict(status='passed', numerical_files=len(numerical),
                         latex_files=len(latexpaths),
                         package_files=len(members) + 1,
                         archives=manifest['archives']), ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
