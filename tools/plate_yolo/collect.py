#!/usr/bin/env python3
"""Copy a spaced, deduplicated sample of Gazebo camera frames for labeling."""
from pathlib import Path
import argparse, hashlib, shutil

ap=argparse.ArgumentParser()
ap.add_argument('--results',type=Path,default=Path.home()/'aic_ws/aic_results')
ap.add_argument('--raw',type=Path,default=Path.home()/'aic_ws/plate_yolo_work/raw')
ap.add_argument('--per-session',type=int,default=35)
a=ap.parse_args()
if a.per_session<=0: ap.error('--per-session must be positive')
if not a.results.is_dir(): ap.error('Results directory not found: '+str(a.results))
output=a.raw/'images'; output.mkdir(parents=True,exist_ok=True)
seen=set(); count=0
for f in output.glob('*'):
    if f.is_file(): seen.add(hashlib.sha256(f.read_bytes()).hexdigest())
sessions=sorted(p for p in a.results.glob('vision_*') if p.is_dir())
for session in sessions:
    imgs=sorted(x for x in session.iterdir() if x.suffix.lower() in ('.jpg','.jpeg','.png') and x.name.startswith('camera_'))
    if not imgs: continue
    indices=sorted(set(round(i*(len(imgs)-1)/max(1,min(len(imgs),a.per_session)-1)) for i in range(min(len(imgs),a.per_session))))
    for i in indices:
        src=imgs[i]
        data=src.read_bytes(); key=hashlib.sha256(data).hexdigest()
        if key in seen:continue
        seen.add(key)
        dst=output / f'{session.name}__{src.name}'
        if dst.exists():continue
        shutil.copy2(src,dst);count+=1
print(f'Added {count} non-identical camera frames from {len(sessions)} sessions. Total raw images: {len(list(output.glob("*")))}')
print('Open images and label ONLY the frames where the real plate is visible. Use annotate.py.')
