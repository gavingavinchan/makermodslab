# Copyright 2026 MakerMods. All rights reserved.
# Licensed under the Apache License, Version 2.0; see repository LICENSE.
"""Inventory final package and four approved mesh dependencies by exact SHA256."""
from pathlib import Path
import hashlib,json
HERE=Path(__file__).resolve().parent
OUT=HERE/'results'
def sha(p):
    h=hashlib.sha256()
    with p.open('rb') as f:
        for block in iter(lambda:f.read(1024*1024),b''):h.update(block)
    return h.hexdigest()
def include(p):
    rel=p.relative_to(HERE)
    return p.is_file() and '__pycache__' not in rel.parts and p.suffix not in ['.log','.pyc','.blend1'] and p.name not in ['rear_radius_fit.json','manifest.json','SHA256SUMS','sync_verification.json','.DS_Store']
files=sorted([p for p in HERE.rglob('*') if include(p)]+[HERE.parent/'assets/propeller_corrected'/f'blade_{i}.stl' for i in range(4)])
entries=[]
for p in files:
    try:name=str(p.relative_to(HERE))
    except ValueError:name='../assets/propeller_corrected/'+p.name
    size=p.stat().st_size
    assert size<100*1024*1024, (name,size,'exceeds100MiB')
    entries.append(dict(path=name,bytes=size,sha256=sha(p)))
report=dict(schema='viscous-cinematic-manifest-v1',files=entries,total_bytes=sum(x['bytes'] for x in entries),file_count=len(entries),scope='Selected cinematic source, assets, reports and media plus four existing approved propeller STLs; excludes ignored logs/caches, manifest itself and synchronization receipt')
(OUT/'manifest.json').write_text(json.dumps(report,indent=2)+'\n')
(OUT/'SHA256SUMS').write_text(''.join(f"{x['sha256']}  {x['path']}\n" for x in entries))
print(json.dumps({k:v for k,v in report.items() if k!='files'},indent=2))
