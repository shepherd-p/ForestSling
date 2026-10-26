"""Structural checks, final PDF metadata and page images for visual review."""
from pathlib import Path
import hashlib,json,re,subprocess,shutil
from pypdf import PdfReader,PdfWriter
import pdfplumber
from PIL import Image,ImageDraw

ROOT=Path(__file__).resolve().parents[1];M=ROOT/'manuscript';QA=ROOT/'qa'
POPPLER=Path('C:/Users/Stephen/.cache/codex-runtimes/codex-primary-runtime/dependencies/native/poppler/Library/bin')
source={p.name:p.read_text(encoding='utf-8') for p in M.glob('*.tex')}
text='\n'.join(source.values())
title=re.search(r'\\title\{([^}]+)\}',source['main.tex'])[1]
abstract=re.search(r'\\begin\{abstract\}(.*?)\\end\{abstract\}',source['main.tex'],re.S)[1]
word_count=len(re.findall(r"[A-Za-z0-9]+(?:[-'][A-Za-z0-9]+)*",abstract))
assert word_count<=250,word_count
cites=set()
for group in re.findall(r'\\cite\w*\{([^}]+)\}',text):cites.update(group.split(','))
references=set(re.findall(r'\\bibitem\{([^}]+)\}',text));assert cites==references
labels=re.findall(r'\\label\{([^}]+)\}',text)
refs=re.findall(r'\\(?:eqref|ref)\{([^}]+)\}',text)
assert len(labels)==len(set(labels)) and set(refs)<=set(labels)
log=(QA/'main.log').read_text(encoding='utf-8',errors='replace')
assert not re.search(r'Overfull|(?:Citation|Reference).*undefined|There were undefined',log)
highlights=(ROOT/'editorial/highlights.txt').read_text(encoding='utf-8').splitlines()
assert 3<=len(highlights)<=5 and all(len(line)<=85 for line in highlights)
for phrase in ('proposed future planner','author-review draft','P1--P6','reserved result','draft completion','TODO','TBD'):
    assert not re.search(r'\b'+re.escape(phrase)+r'\b',text,re.I),phrase
metadata=json.loads((ROOT/'supplementary/bibliography/METADATA_CHECK.json').read_text(encoding='utf-8'))
assert len(metadata)==len(references) and all(row['verified'] for row in metadata)
writer=PdfWriter(clone_from=str(QA/'main.pdf'))
writer.add_metadata({'/Title':title,'/Author':'Fangwei Peng; Juntong Qi; Yan Peng; Mingming Wang; Chong Wu',
 '/Subject':'Cooperative aerial transport with visual and tension sensing, inertial identification and payload reorientation'})
english=ROOT/'ForestSling_AST_Manuscript.pdf'
with english.open('wb') as f:writer.write(f)
chinese=ROOT/'ForestSling_AST_Chinese_Manuscript.pdf'
shutil.copy2(QA/'chinese/ForestSling_AST_Chinese_Manuscript.pdf',chinese)
pages=QA/'english';pages.mkdir(exist_ok=True)
subprocess.run([str(POPPLER/'pdftoppm.exe'),'-r','125','-png',str(english),str(pages/'page')],check=True,capture_output=True)
report=dict(abstract_words=word_count,figures=len(re.findall(r'\\begin\{figure\*?\}',text)),
 tables=len(re.findall(r'\\begin\{table\*?\}',text)),bibliography_records=len(references),
 references_resolved=True,overfull_boxes=[],highlight_character_counts=list(map(len,highlights)),pdfs=[])
for name,path,folder in [('English',english,pages),('Chinese',chinese,QA/'chinese')]:
    with pdfplumber.open(path) as pdf:
        outside=[];extracted=[]
        for i,page in enumerate(pdf.pages,1):
            extracted.append(page.extract_text() or '')
            for char in page.chars:
                if char['text'].strip() and (char['x0']<9 or char['x1']>page.width-9 or char['top']<9 or char['bottom']>page.height-9):
                    outside.append(dict(page=i,text=char['text'],bbox=[char['x0'],char['top'],char['x1'],char['bottom']]))
        fulltext='\n'.join(extracted)
        (QA/(name.lower()+'_text.txt')).write_text(fulltext,encoding='utf-8')
        assert not outside,outside[:10]
        assert '\ufffd' not in fulltext and '??' not in fulltext
        report['pdfs'].append(dict(language=name,pages=len(pdf.pages),sha256=hashlib.sha256(path.read_bytes()).hexdigest(),text_outside_page=outside))
    images=sorted((p for p in folder.glob('page*.png') if int(re.search(r'(\d+)\.png$',p.name)[1])<=len(pdf.pages)),key=lambda p:int(re.search(r'(\d+)\.png$',p.name)[1]))
    for start in range(0,len(images),4):
        sheet=Image.new('RGB',(1400,2000),'#d9dee2');draw=ImageDraw.Draw(sheet)
        for k,path in enumerate(images[start:start+4]):
            img=Image.open(path).convert('RGB');img.thumbnail((680,950));x=(k%2)*700+10;y=(k//2)*1000+30
            sheet.paste(img,(x,y));draw.text((x,y-20),f'{name} page {start+k+1}',fill='black')
        sheet.save(QA/f'{name.lower()}_sheet_{start//4+1:02}.png')
report['status']='structural_checks_passed_visual_review_pending'
(QA/'QUALITY_REPORT.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(report,ensure_ascii=False,indent=2))
