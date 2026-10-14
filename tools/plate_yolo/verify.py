#!/usr/bin/env python3
"""Offline full-pipeline test: YOLO plate rectangles + existing 3-plate HOG/SVM classification."""
from pathlib import Path
import argparse, json, importlib.util, sys

ap=argparse.ArgumentParser()
ap.add_argument('--weights',type=Path,required=True)
ap.add_argument('--images',type=Path,required=True,help='an image or directory of images, ideally withheld from training')
ap.add_argument('--workspace',type=Path,default=Path.home()/'aic_ws')
ap.add_argument('--output',type=Path,default=Path.home()/'aic_ws/plate_yolo_work/verification')
ap.add_argument('--conf',type=float,default=.25)
a=ap.parse_args()
if not a.weights.is_file():ap.error('Weights not found: '+str(a.weights))
if not a.images.exists():ap.error('Images not found: '+str(a.images))
try:
    import cv2
    from ultralytics import YOLO
except ImportError as e: ap.error(f'Cannot load inference dependencies: {e}')
file=a.workspace/'src/robot_description/scripts/aic_team_vision_core.py'
model_dir=a.workspace/'src/robot_description/inspection_models'
if not file.is_file() or not model_dir.is_dir():ap.error('Missing existing SVM classifier and models. This package does not train the character-recognizer.')
spec=importlib.util.spec_from_file_location('aic_team_vision_core',file)
core=importlib.util.module_from_spec(spec);sys.modules['aic_team_vision_core']=core;spec.loader.exec_module(core)
core.MODELS=model_dir
_,_,svm,meta=core.load_models()
detector=YOLO(str(a.weights))
class_names={int(k):str(v) for k,v in detector.names.items()}
if len(class_names)!=1 or 'plate' not in class_names[0].lower():
    ap.error('These weights are not a trained SINGLE-CLASS plate detector; detected classes: '+repr(class_names))
paths=sorted(x for x in a.images.iterdir() if x.suffix.lower() in ('.jpg','.png','.jpeg')) if a.images.is_dir() else [a.images]
a.output.mkdir(parents=True,exist_ok=True)
results=[]
for file in paths:
    frame=cv2.imread(str(file))
    if frame is None:continue
    row={'image':str(file),'plate_predictions':[],'recognizer':'EXISTING_FIXED_3_PLATE_SVM_NOT_GENERAL_OCR'}
    pred=detector.predict(frame,imgsz=640,conf=a.conf,device='cpu',verbose=False)[0]
    for box in pred.boxes:
        x1,y1,x2,y2=[int(v) for v in box.xyxy[0].tolist()]
        x1=max(0,min(x1,frame.shape[1]-1));x2=max(0,min(x2,frame.shape[1]));y1=max(0,min(y1,frame.shape[0]-1));y2=max(0,min(y2,frame.shape[0]))
        if x2<=x1 or y2<=y1:continue
        crop=frame[y1:y2,x1:x2]
        index=core.classify_patch(svm,core.PLATE_HOG,crop,core.PLATE_SIZE)
        label=meta['plate_labels'][index] if 0<=index<len(meta['plate_labels']) else None
        score=float(box.conf[0])
        row['plate_predictions'].append({'bbox':[x1,y1,x2-x1,y2-y1],'yolo_confidence':round(score,4),'plate_text':label,'text_verified':False})
        cv2.rectangle(frame,(x1,y1),(x2,y2),(0,255,0),2)
        cv2.putText(frame,f'PLATE {score:.2f}',(x1,max(15,y1-6)),cv2.FONT_HERSHEY_SIMPLEX,.55,(0,255,0),2)
    results.append(row)
    cv2.imwrite(str(a.output/file.name),frame)
(a.output/'results.json').write_text(json.dumps(results,ensure_ascii=False,indent=2),encoding='utf-8')
print(f'Processed {len(results)} images. Review annotated images AND compare plate_text with ground-truth images manually.')
print('Results:',a.output/'results.json')
