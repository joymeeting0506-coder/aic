#!/usr/bin/env python3
"""Mix YOLO synthetic TRAIN images with real Gazebo TRAIN images; validation REAL ONLY.

Requires separate datasets created by generate_synthetic.py and split.py.
"""
import argparse
from pathlib import Path
from shutil import copy2
ap=argparse.ArgumentParser(description=__doc__)
ap.add_argument('--synthetic',type=Path,default=Path.home()/'aic_ws/plate_yolo_work/dataset_synthetic')
ap.add_argument('--real',type=Path,default=Path.home()/'aic_ws/plate_yolo_work/dataset_real')
ap.add_argument('--out',type=Path,default=Path.home()/'aic_ws/plate_yolo_work/dataset_mixed')
a=ap.parse_args()
out=a.out.expanduser().resolve()
for split in ('train','val'):
    for kind in ('images','labels'):
        p=out/kind/split
        if p.exists() and any(p.iterdir()): ap.error(f'Output not empty: {p}')
        p.mkdir(parents=True,exist_ok=True)

def add_dataset(root,split,prefix):
    ims=root/'images'/split;labs=root/'labels'/split
    if not ims.is_dir() or not labs.is_dir(): ap.error(f'Missing {ims} or {labs}')
    n=0; positive=0
    for src in sorted(ims.iterdir()):
        if src.suffix.lower() not in ('.jpg','.png','.jpeg'):continue
        label=labs/(src.stem+'.txt')
        if not label.is_file(): ap.error('Image without YOLO label: '+str(src))
        copy2(src,out/'images'/split/(prefix+'_'+src.name))
        copy2(label,out/'labels'/split/(prefix+'_'+src.stem+'.txt'))
        n+=1
        positive+=bool(label.read_text().strip())
    if n<1 or positive<1:ap.error(f'{root} {split} lacks any positive images')
    return n,positive
s=add_dataset(a.synthetic,'train','synthetic')
r=add_dataset(a.real,'train','real')
v=add_dataset(a.real,'val','real')
(out/'plate.yaml').write_text(f'path: {out}\ntrain: images/train\nval: images/val\nnames:\n  0: license_plate\n',encoding='utf8')
print('Training (synthetic/real)',s,r,'Validation REAL ONLY',v)
print('YOLO data:',out/'plate.yaml')
print('NOTE: your real validation images must be from an independent Gazebo recording session.')
