#!/usr/bin/env python3
"""Build a YOLO dataset. Sessions remain disjoint between train and validation."""
from pathlib import Path
import argparse, hashlib, math, random, shutil
ap=argparse.ArgumentParser()
ap.add_argument('--raw',type=Path,default=Path.home()/'aic_ws/plate_yolo_work/raw')
ap.add_argument('--out',type=Path,default=Path.home()/'aic_ws/plate_yolo_work/dataset')
ap.add_argument('--seed',type=int,default=42)
ap.add_argument('--allow-frame-split',action='store_true',help='Demonstration only: same-session frames in train+val inflate metrics')
a=ap.parse_args()
imgs=a.raw/'images'; labs=a.raw/'labels'
items=[p for p in sorted(imgs.glob('*')) if p.suffix.lower() in ('.jpg','.jpeg','.png') and (labs/(p.stem+'.txt')).exists()]
if len(items)<2:ap.error('Need at least 2 manually annotated images (try 40+ across >=2 simulation sessions).')
# Avoid byte-identical frames even if they had different names.
uniq=[];seen=set()
for p in items:
    h=hashlib.sha256(p.read_bytes()).hexdigest()
    if h not in seen:seen.add(h);uniq.append(p)
items=uniq
session=lambda p:p.name.split('__',1)[0] if '__' in p.name else p.name.split('_camera_',1)[0]
groups={}
for p in items:groups.setdefault(session(p),[]).append(p)
rng=random.Random(a.seed)
if len(groups)<2:
    if not a.allow_frame_split:
        ap.error('Only one recording session. Record another distinct plate viewing session, or use --allow-frame-split (metrics NOT reliable).')
    print('WARNING: FRAME-BASED SPLIT IS LEAKY; validation is NOT a generalization estimate.')
    copy=items[:];rng.shuffle(copy);size=max(1,round(len(copy)*.2));val=set(copy[:size]);train=set(copy[size:])
else:
    names=list(groups);rng.shuffle(names)
    val_names=set(names[:max(1,round(.2*len(names)))])
    val={p for k in val_names for p in groups[k]};train=set(items)-val
    print('Training sessions:',sorted(set(groups)-val_names))
    print('Validation sessions:',sorted(val_names))
for split in ('train','val'):
    for kind in ('images','labels'):
        dest=a.out/kind/split
        if dest.is_dir() and any(dest.iterdir()):ap.error('Output dataset already contains files; choose a new --out or manually delete old dataset to avoid stale images: '+str(dest))
for split,batch in (('train',train),('val',val)):
    (a.out/'images'/split).mkdir(parents=True,exist_ok=True)
    (a.out/'labels'/split).mkdir(parents=True,exist_ok=True)
    for p in sorted(batch):
        shutil.copy2(p,a.out/'images'/split/p.name)
        shutil.copy2(labs/(p.stem+'.txt'),a.out/'labels'/split/(p.stem+'.txt'))
    positives=sum(bool((labs/(p.stem+'.txt')).read_text().strip()) for p in batch)
    print(split,'total',len(batch),'with plate',positives,'empty/negative',len(batch)-positives)
    if positives==0: ap.error(f'{split} set has no positive plate boxes. Add actual plate images from more sessions.')
(a.out/'plate.yaml').write_text(f'path: {a.out.resolve()}\ntrain: images/train\nval: images/val\nnames:\n  0: license_plate\n',encoding='utf8')
print('Dataset ready:',a.out/'plate.yaml')
