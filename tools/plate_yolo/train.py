#!/usr/bin/env python3
"""Train and validate single-class plate YOLO. Training requires annotated data."""
from pathlib import Path
import argparse,json
ap=argparse.ArgumentParser()
ap.add_argument('--data',type=Path,default=Path.home()/'aic_ws/plate_yolo_work/dataset/plate.yaml')
ap.add_argument('--model',default='yolo11n.pt',help='YOLO pretrained detection weight or local .pt (downloads if name is online)')
ap.add_argument('--epochs',type=int,default=60)
ap.add_argument('--batch',type=int,default=8)
ap.add_argument('--imgsz',type=int,default=640)
ap.add_argument('--device',default='cpu',help='cpu or 0 for CUDA GPU')
ap.add_argument('--workers',type=int,default=0)
ap.add_argument('--project',type=Path,default=Path.home()/'aic_ws/plate_yolo_work/runs')
a=ap.parse_args()
if not a.data.exists():ap.error('Dataset not found; run annotate.py and split.py first.')
if a.epochs<1 or a.imgsz<128 or a.batch<1:ap.error('epochs, batch and imgsz must be positive')
try:from ultralytics import YOLO
except ImportError:ap.error('pip install ultralytics into the active training Python environment')
model=YOLO(a.model)
result=model.train(data=str(a.data.resolve()),epochs=a.epochs,imgsz=a.imgsz,batch=a.batch,workers=a.workers,device=a.device,project=str(a.project),name='plate_detect',exist_ok=False,patience=min(20,a.epochs),seed=42)
# Do not claim accuracy without metrics. best.pt is usable only after a real training run.
best=Path(model.trainer.best)
print('BEST_WEIGHTS:',best)
metrics=YOLO(str(best)).val(data=str(a.data.resolve()),imgsz=a.imgsz,device=a.device,workers=a.workers)
report={'best_weights':str(best),'data':str(a.data.resolve()),'mAP50':float(metrics.box.map50),'mAP50_95':float(metrics.box.map),'precision':float(metrics.box.mp),'recall':float(metrics.box.mr),'note':'These are detection-box metrics on the chosen validation split, not Chinese-plate character recognition accuracy.'}
report_path=best.parent.parent/'plate_validation.json';report_path.write_text(json.dumps(report,indent=2,ensure_ascii=False),encoding='utf8')
print('Metrics:',json.dumps(report,ensure_ascii=False,indent=2))
print('Validation record:',report_path)
