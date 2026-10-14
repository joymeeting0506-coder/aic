"""Two-stage YOLO inference. Never substitute a known plate number for model output."""
import os
from pathlib import Path
os.environ.setdefault('YOLO_CONFIG_DIR',str(Path(__file__).resolve().parent/'settings'))
os.environ.setdefault('MPLCONFIGDIR',str(Path(__file__).resolve().parent/'settings/matplotlib'))
import argparse
import json
import cv2
import numpy as np
from PIL import Image,ImageDraw,ImageFont
from ultralytics import YOLO
from common import ROOT,PROVINCES,LETTERS,ALNUM,CHARS,read_image,crop_plate

def decode_characters(result):
    boxes=[]
    for b in result.boxes:
        xy=b.xyxy[0].cpu().tolist()
        boxes.append(dict(char=result.names[int(b.cls.item())],confidence=float(b.conf.item()),xyxy=xy))
    ignored=[]
    if len(boxes)>7:
        median_height=float(np.median([b['xyxy'][3]-b['xyxy'][1] for b in boxes]))
        # A low-confidence, short mark between characters can be the plate separator.
        ignored=[b for b in boxes if b['confidence']<.5 and b['xyxy'][3]-b['xyxy'][1]<.75*median_height]
        boxes=[b for b in boxes if b not in ignored]
    boxes.sort(key=lambda b:(b['xyxy'][0]+b['xyxy'][2])/2)
    text=''.join(b['char'] for b in boxes)
    valid=(len(text)==7 and text[0] in PROVINCES and text[1] in LETTERS and all(c in ALNUM for c in text[2:]))
    return dict(text=text,format_valid=valid,needs_review=not valid or min((b['confidence'] for b in boxes),default=0)<.5,
                min_character_confidence=min((b['confidence'] for b in boxes),default=0),characters=boxes,ignored_small_marks=ignored)

class PlateReader:
    def __init__(self,detector=ROOT/'models/plate_detector.pt',characters=ROOT/'models/characters.pt',device='0'):
        self.detector=YOLO(str(detector)); self.characters=YOLO(str(characters)); self.device=device
        if list(self.detector.names.values())!=['license_plate']:raise ValueError('Wrong plate detector classes')
        if ''.join(self.characters.names[i] for i in range(len(CHARS)))!=CHARS:raise ValueError('Wrong character alphabet')

    def read_crop(self,crop):
        if crop.size==0:raise ValueError('Empty plate crop')
        r=self.characters.predict(crop,imgsz=320,conf=.15,iou=.35,agnostic_nms=True,max_det=12,device=self.device,verbose=False)[0]
        primary=decode_characters(r);primary['inference_variant']='320'
        if primary['needs_review']:
            result=self.characters.predict(crop,imgsz=384,conf=.15,iou=.35,agnostic_nms=True,max_det=12,device=self.device,verbose=False)[0]
            retry=decode_characters(result);retry['inference_variant']='384_retry'
            score=lambda r:(r['format_valid'],r['min_character_confidence'])
            if score(retry)>score(primary):
                retry['initial_text']=primary['text']
                return retry
        return primary

    def __call__(self,frame):
        pred=self.detector.predict(frame,imgsz=640,conf=.25,iou=.5,max_det=20,device=self.device,verbose=False)[0]
        output=[]
        for b in pred.boxes:
            xy=b.xyxy[0].cpu().tolist()
            crop,crop_box=crop_plate(frame,xy)
            rec=self.read_crop(crop)
            rec.update(plate_xyxy=xy,plate_confidence=float(b.conf.item()),crop_xyxy=crop_box)
            output.append(rec)
        return output

def annotate(frame,records):
    im=Image.fromarray(cv2.cvtColor(frame,cv2.COLOR_BGR2RGB)); d=ImageDraw.Draw(im)
    font_path='C:/Windows/Fonts/msyh.ttc'
    if not Path(font_path).exists():
        font_path='/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
    font=ImageFont.truetype(font_path,18)
    for rec in records:
        x1,y1,x2,y2=rec['plate_xyxy']; color=(255,180,0) if rec['needs_review'] else (0,255,80)
        d.rectangle((x1,y1,x2,y2),outline=color,width=2)
        text=rec['text'] or '[no characters]'
        d.text((max(0,x1),max(0,y1-25)),text,font=font,fill=color,stroke_width=1,stroke_fill=(0,0,0))
    return cv2.cvtColor(np.asarray(im),cv2.COLOR_RGB2BGR)

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--source',type=Path,required=True)
    ap.add_argument('--out',type=Path,default=ROOT/'predictions')
    ap.add_argument('--device',default='0')
    ap.add_argument('--detector',type=Path,default=ROOT/'models/plate_detector.pt')
    ap.add_argument('--characters',type=Path,default=ROOT/'models/characters.pt')
    args=ap.parse_args(); args.out.mkdir(parents=True,exist_ok=True)
    paths=sorted(p for p in args.source.iterdir() if p.suffix.lower() in {'.jpg','.png','.jpeg','.bmp'}) if args.source.is_dir() else [args.source]
    if not paths:raise ValueError('No images found')
    reader=PlateReader(args.detector,args.characters,args.device); records=[]
    for path in paths:
        frame=read_image(path); plates=reader(frame)
        records.append(dict(image=str(path),plates=plates))
        cv2.imwrite(str(args.out/(path.stem+'.jpg')),annotate(frame,plates))
        print(json.dumps(records[-1],ensure_ascii=False),flush=True)
    (args.out/'predictions.json').write_text(json.dumps(records,ensure_ascii=False,indent=2),encoding='utf-8')

if __name__=='__main__':main()
