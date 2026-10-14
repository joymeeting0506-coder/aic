#!/usr/bin/env python3
"""Safely integrate optional plate YOLO into the audited v5.6 vision pipeline.

DEFAULT dry-run (no modifications). --apply creates backups, then patches exact anchors.
Do not use against unrelated versions; it will refuse unexpected code.
"""
from pathlib import Path
import argparse,difflib,shutil,time

ap=argparse.ArgumentParser()
ap.add_argument('--workspace',type=Path,default=Path.home()/'aic_ws')
ap.add_argument('--apply',action='store_true')
a=ap.parse_args()
root=a.workspace
vision=root/'src/robot_description/scripts/aic_v5_vision.py'
launch=root/'src/robot_description/launch/aic_v5.launch.py'
shell=root/'tools/aic_one_run.sh'

# Indentation matches the v5.6 audited text (2026-10-13). User-owned code is
# never forcibly replaced when anchor matching fails.
def once(s,needle,addition,what):
    if addition in s:
        return s
    if s.count(needle)!=1:raise RuntimeError(f'{what}: anchor missing or not unique; do not auto-apply; inspect manually')
    return s.replace(needle,addition,1)

VISION_PLATE_OLD="""                    if name.startswith('PLATE_'):
                        found=self.team.detect_plates(frame,self.plate_model,self.meta['plate_labels'])
                        output['plate_predictions']=[{'text':txt,'bbox':[x,y,w,h],
                            'score_note':'TEMPLATE_MODEL_NOT_CALIBRATED'}
                            for x,y,w,h,txt,_ in found]
                        output['ocr']='FIXED_SIMULATION_PLATE_CLASSIFIER_NOT_GENERAL_OCR'
"""
VISION_PLATE_NEW="""                    if name.startswith('PLATE_'):
                        if self.plate_yolo is not None:
                            predictions=[]
                            inference=self.plate_yolo.predict(source=frame,conf=.25,imgsz=640,
                                                              device='cpu',verbose=False)[0]
                            for b in inference.boxes:
                                x1,y1,x2,y2=[int(q) for q in b.xyxy[0].tolist()]
                                x1=max(0,min(x1,frame.shape[1]-1))
                                x2=max(0,min(x2,frame.shape[1]))
                                y1=max(0,min(y1,frame.shape[0]-1))
                                y2=max(0,min(y2,frame.shape[0]))
                                if x2<=x1 or y2<=y1:
                                    continue
                                patch=frame[y1:y2,x1:x2]
                                ix=self.team.classify_patch(self.plate_model,self.team.PLATE_HOG,
                                                            patch,self.team.PLATE_SIZE)
                                text=self.meta['plate_labels'][ix] if 0<=ix<len(self.meta['plate_labels']) else None
                                predictions.append({'text':text,'bbox':[x1,y1,x2-x1,y2-y1],
                                    'yolo_confidence':round(float(b.conf[0]),4),
                                    'score_note':'TEXT_SVM_UNCALIBRATED_NOT_GENERAL_OCR'})
                            output['plate_predictions']=predictions
                            output['plate_detector']='YOLO_CUSTOM_PLATE'
                            output['ocr']='YOLO_DETECT_PLUS_FIXED_SIMULATION_SVM_TEXT_CLASSIFIER'
                            if not predictions:
                                output['status']='NO_PLATE_DETECTED'
                        else:
                            found=self.team.detect_plates(frame,self.plate_model,self.meta['plate_labels'])
                            output['plate_predictions']=[{'text':txt,'bbox':[x,y,w,h],
                                'score_note':'TEMPLATE_MODEL_NOT_CALIBRATED'}
                                for x,y,w,h,txt,_ in found]
                            output['plate_detector']='HSV_COLOR_FALLBACK'
                            output['ocr']='FIXED_SIMULATION_PLATE_CLASSIFIER_NOT_GENERAL_OCR'
                            if not found:
                                output['status']='NO_PLATE_DETECTED'
"""

def change_vision(s):
    marker="ap.add_argument('--plate-yolo-weights',default=''"
    if marker not in s:
        s=once(s,"    ap.add_argument('--yolo-weights',default='',help='Optional already-downloaded model file. Never silently downloads.')\n",
               "    ap.add_argument('--yolo-weights',default='',help='Optional already-downloaded model file. Never silently downloads.')\n"
               "    ap.add_argument('--plate-yolo-weights',default='',help='Custom YOLO plate detector. Empty keeps old HSV+SVM fallback.')\n",'vision CLI')
    if 'self.plate_yolo=None' not in s:
        s=once(s,'            self._load_inspection_models()\n',
"""            self._load_inspection_models()
            self.plate_yolo=None
            if args.plate_yolo_weights:
                p=Path(args.plate_yolo_weights).expanduser()
                if not p.is_file():raise RuntimeError('Plate YOLO weights missing: '+str(p))
                try:
                    from ultralytics import YOLO
                except ImportError:
                    raise RuntimeError('Ultralytics unavailable in ROS vision Python environment')
                self.plate_yolo=YOLO(str(p))
                class_names={int(k):str(v) for k,v in self.plate_yolo.names.items()}
                if len(class_names)!=1 or 'plate' not in class_names[0].lower():
                    raise RuntimeError('Plate YOLO requires ONE license_plate class: '+str(class_names))
                self.get_logger().info('Custom plate YOLO loaded: '+str(p))
""",'vision YOLO load')
    if VISION_PLATE_NEW not in s:s=once(s,VISION_PLATE_OLD,VISION_PLATE_NEW,'vision inspection branch')
    if "if output['status']=='IMAGE_SAVED':" not in s:
        s=once(s,"                    output['status']='MODEL_INFERENCE_COMPLETED'\n",
"                    if output['status']=='IMAGE_SAVED':\n                        output['status']='MODEL_INFERENCE_COMPLETED'\n",'vision outcome status')
    return s

def change_launch(s):
    if "['--plate-yolo-weights=',LaunchConfiguration('plate_yolo_weights')]" not in s:
        s=once(s,"                   ['--yolo-weights=',LaunchConfiguration('yolo_weights')],\n",
               "                   ['--yolo-weights=',LaunchConfiguration('yolo_weights')],\n"
               "                   ['--plate-yolo-weights=',LaunchConfiguration('plate_yolo_weights')],\n",'launch vision args')
    if "DeclareLaunchArgument('plate_yolo_weights'" not in s:
        s=once(s,"        DeclareLaunchArgument('yolo_weights',default_value='',\n            description='Optional local YOLO weights; empty means NOT_CONFIGURED'),\n",
               "        DeclareLaunchArgument('yolo_weights',default_value='',\n            description='Optional local YOLO weights; empty means NOT_CONFIGURED'),\n"
               "        DeclareLaunchArgument('plate_yolo_weights',default_value='',\n            description='Single-class trained YOLO plate detector; empty keeps HSV+SVM fallback'),\n",'launch declare args')
    return s

def change_shell(s):
    if 'AIC_PLATE_YOLO_WEIGHTS' not in s:
        s=once(s,'python3 "$VISION" --snapshot-interval 0 --ros-args -p use_sim_time:=true \\\n',
"""VISION_ARGS=(--snapshot-interval 0)
if [[ -n "${AIC_PLATE_YOLO_WEIGHTS:-}" ]]; then
    VISION_ARGS+=(--plate-yolo-weights "$AIC_PLATE_YOLO_WEIGHTS")
fi
python3 "$VISION" "${VISION_ARGS[@]}" --ros-args -p use_sim_time:=true \\
""",'one run script plate arg')
    return s

changes=[]
for p,fn in [(vision,change_vision),(launch,change_launch),(shell,change_shell)]:
    if not p.is_file():raise SystemExit('Missing '+str(p)+'. No files modified.')
    old=p.read_text(encoding='utf8')
    try:new=fn(old)
    except Exception as exc:
        raise SystemExit(f'Refusing to patch {p}: {exc}. No files modified.')
    changes.append((p,old,new))
for p,old,new in changes:
    print(f'== {p.relative_to(root)}: '+('UNCHANGED' if old==new else 'CHANGED')+' ==')
    diff=''.join(difflib.unified_diff(old.splitlines(True),new.splitlines(True),fromfile='before',tofile='after'))
    if diff: print(diff[:12000])
if not a.apply:
    print('DRY RUN: No changes made. Run again with --apply after reviewing the diff.')
else:
    tag=time.strftime('%Y%m%d_%H%M%S')
    for p,old,new in changes:
        if old==new:continue
        backup=p.with_name(p.name+f'.plate_backup_{tag}')
        shutil.copy2(p,backup)
        p.write_text(new,encoding='utf8')
        print('Backed up:',backup)
    print('Patched. Now: python3 -m py_compile <visual source>; colcon build --symlink-install --packages-select robot_description')
