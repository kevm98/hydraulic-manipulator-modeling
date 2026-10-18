"""Validate retained files and portable model references without MATLAB."""
from pathlib import Path
import csv
import hashlib
import json
import math
import re
import sys
import zipfile
import xml.etree.ElementTree as ET
from urllib.parse import unquote

ROOT=Path(__file__).resolve().parents[1]
errors=[]
def check(ok,msg):
    if not ok:errors.append(msg)
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()

mapping=json.loads((ROOT/'docs/original/migration-map.json').read_text())
for row in mapping['files']:
    p=ROOT/row['target'];check(p.is_file(),'Missing retained file: '+row['target'])
    if not p.exists():continue
    if not row['modified']:check(sha(p)==row['sha256'],'Changed retained source: '+row['target'])
    elif p.suffix=='.slx':
        with zipfile.ZipFile(p) as z:
            expected=row['zip_member_sha256']
            check(set(z.namelist())==set(expected),'Changed SLX members: '+row['target'])
            for name,digest in expected.items():
                check(hashlib.sha256(z.read(name)).hexdigest()==digest,'Changed model member: '+row['target']+' / '+name)
supp=mapping['supplementary_report']
check(sha(ROOT/supp['target'])==supp['sha256'],'Changed supplementary original report')

models=list((ROOT/'models').rglob('*.slx'))
for p in models:
    with zipfile.ZipFile(p) as z:
        check(z.testzip() is None,'Invalid SLX ZIP: '+p.name)
        for member in z.namelist():
            if not member.endswith('.xml'):continue
            try:node=ET.fromstring(z.read(member))
            except ET.ParseError:continue
            for element in node.iter('P'):
                field=element.attrib.get('Name');v=element.text or ''
                if field=='ExtGeomFileName' and v:
                    check(not re.match(r'^[A-Za-z]:',v),'Absolute Windows model mesh: '+v)
                    check((ROOT/v).is_file(),'Missing model mesh: '+v)
                if field=='FileName' and v.startswith('data/references/'):
                    check((ROOT/v).is_file(),'Missing Signal Editor file: '+v)

for p in (ROOT/'robot/urdf').glob('*.urdf'):
    node=ET.parse(p).getroot()
    for mesh in node.iter('mesh'):
        v=mesh.attrib.get('filename','');check((p.parent/v).is_file(),'Missing URDF mesh: '+v)

for name,count in [('reference',3991),('measured',3550)]:
    p=ROOT/f'data/figure-traces/task_path_{name}.csv'
    with p.open() as f:
        r=list(csv.reader(f));check(r[0]==['x_m','y_m','z_m'],'Invalid path units: '+name)
        check(len(r)-1==count,'Wrong curve count: '+name)
        check(all(len(v)==3 and all(math.isfinite(float(a)) for a in v) for v in r[1:]),'Invalid numeric path: '+name)
with (ROOT/'data/figure-traces/bulk_modulus_summary.csv').open() as f:summary=list(csv.DictReader(f))
check([float(r['j3_delay_ms']) for r in summary]==[186.3,168.8,149.9,152.4],'Unexpected joint-3 series')
check([float(r['ee_mean_mm']) for r in summary]==[211.3,215.4,229.1,228.8],'Unexpected mean-error series')
for name in ['bulk-modulus-sensitivity','task-space-path','coupled-architecture']:
    for ext in ['pdf','svg','png']:check((ROOT/f'docs/report/figures/{name}.{ext}').is_file(),'Missing figure: '+name+'.'+ext)

for p in [ROOT/'README.md',*list((ROOT/'docs').glob('*.md'))]:
    for link in re.findall(r'\]\(([^)]+)\)',p.read_text()):
        if link.startswith(('https:','http:','#')):continue
        check((p.parent/unquote(link.split('#')[0])).exists(),'Broken link in '+str(p.relative_to(ROOT))+': '+link)
tex=(ROOT/'docs/report/main.tex').read_text()
check('\\documentclass[conference,letterpaper]{IEEEtran}' in tex,'Incorrect report class')
check('426167' not in tex,'Student number in revised report')
check((ROOT/'docs/report/hydraulic_manipulator_report_ieee.pdf').is_file(),'Missing revised PDF')
for p in ROOT.rglob('*'):
    if not p.is_file() or '.git' in p.parts or 'build' in p.parts:continue
    check(p.stat().st_size<100*1024*1024,'File exceeds GitHub blob limit: '+str(p))
    if p.suffix in {'.m','.py','.md','.tex','.yml'}:
        t=p.read_text(errors='replace')
        check(not re.search(r'-----BEGIN (?:OPENSSH |RSA |EC )?PRIVATE KEY-----|gh[pousr]_[A-Za-z0-9]{30,}|glpat-[A-Za-z0-9_-]{16,}',t),'Possible credential: '+str(p.relative_to(ROOT)))
if errors:
    print('\n'.join('FAIL: '+e for e in errors));sys.exit(1)
print('PASS: retained hashes, SLX archives, model/data/URDF paths, trace counts, table values, documentation links, identifiers, file sizes, and credential patterns.')
