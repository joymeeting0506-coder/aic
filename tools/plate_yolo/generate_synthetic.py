#!/usr/bin/env python3
"""AIC: create license-plate YOLO detection training images and labels without manual annotation.

The domain is procedural/synthetic. A real Gazebo-camera validation set is required
before claims of competition readiness. Existing background images MUST NOT include
unlabelled license plates; otherwise detector supervision will be incorrect.
"""
import argparse
import csv
import math
import random
import subprocess
from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

PROVINCES = '京津沪渝苏浙皖闽赣鲁豫鄂湘粤桂琼川贵云陕甘青蒙辽吉黑冀晋宁新藏'
LETTERS = 'ABCDEFGHJKLMNPQRSTUVWXYZ'
ALNUM = 'ABCDEFGHJKLMNPQRSTUVWXYZ0123456789'
EXAMPLES = ['鄂D7B502', '苏AB8Q62', '苏APL12A']


def font_location(family):
    cmd = ['fc-match', '-f', '%{file}', family]
    try:
        f = subprocess.check_output(cmd, text=True, timeout=3).strip()
        if f and Path(f).is_file():
            return f
    except (FileNotFoundError, subprocess.CalledProcessError, subprocess.TimeoutExpired):
        pass
    return None


def plate_image(rng, cn_font, alpha_font):
    plate = Image.new('RGB', (440, 140), (13, 61, rng.randint(155, 203)))
    d = ImageDraw.Draw(plate)
    d.rounded_rectangle((4, 4, 435, 135), radius=10, outline='white', width=4)
    d.ellipse((18, 15, 27, 24), fill=(201, 206, 225))
    d.ellipse((413, 15, 422, 24), fill=(201, 206, 225))
    if rng.random() < .18:
        s = rng.choice(EXAMPLES)
    else:
        s = rng.choice(PROVINCES) + rng.choice(LETTERS) + ''.join(rng.choices(ALNUM, k=5))
    text_positions = [49, 113, 188, 240, 293, 346, 399]
    for idx, (ch, xx) in enumerate(zip(s, text_positions)):
        font = cn_font if idx == 0 else alpha_font
        d.text((xx, 76), ch, font=font, anchor='mm', fill=(246, 250, 252), stroke_width=0)
    d.ellipse((154, 75, 160, 81), fill='white')
    return cv2.cvtColor(np.asarray(plate), cv2.COLOR_RGB2BGR), s


def random_canvas(rng, w, h, backgrounds):
    if backgrounds and rng.random() < .65:
        chosen = rng.choice(backgrounds)
        frame = cv2.imread(str(chosen))
        if frame is not None and frame.shape[0] >= 50 and frame.shape[1] >= 50:
            z = cv2.resize(frame, (w, h), interpolation=cv2.INTER_LINEAR)
            return z
    # Road / buildings / pavement / foliage: simplistic synthetic camera domain.
    sky = np.array(rng.choice([(168, 181, 195), (197, 201, 200), (210, 195, 184), (150, 176, 179)]), dtype=np.float32)
    ground = np.array(rng.choice([(69, 70, 72), (104, 102, 98), (114, 120, 123), (93, 100, 104)]), dtype=np.float32)
    frame = np.zeros((h, w, 3), np.uint8)
    horizon = rng.randint(int(h*.20), int(h*.55))
    frame[:horizon] = sky
    frame[horizon:] = ground
    for _ in range(rng.randint(4, 11)):
        x = rng.randint(-30, w)
        y1 = rng.randint(0, horizon // 3)
        bw = rng.randint(24, 150)
        color = tuple(int(v) for v in rng.choice([(109, 125, 130), (147, 136, 117), (133, 142, 145), (168, 162, 145)]))
        cv2.rectangle(frame, (x, y1), (x+bw, horizon), color, -1)
    for _ in range(rng.randint(0, 7)):
        cy = rng.randint(horizon, h-5)
        cx = rng.randint(0, w-1)
        cv2.line(frame, (cx, cy), (min(w-1, cx+rng.randint(25,110)), cy+rng.randint(-4,4)), (170, 168, 160), rng.randint(1,4))
    return frame


def synthesize(rng, w, h, cn_font, alpha_font, backgrounds, positive=True):
    frame = random_canvas(rng, w, h, backgrounds)
    # Back of a stylized car. It is NOT expected to match the contest car texture.
    cw = rng.randint(int(w*.22), int(w*.47))
    ch = int(cw * rng.uniform(.50, .75))
    x = rng.randint(30, w-cw-30)
    y = rng.randint(int(h*.30), max(int(h*.31),h-ch-20))
    body = rng.choice([(118, 42, 25), (156, 80, 41), (85, 89, 91), (191, 193, 196), (62, 67, 84)])
    top = np.array([[x+cw*.15,y+ch*.04],[x+cw*.85,y+ch*.04],
                    [x+cw*.98,y+ch*.87],[x+cw*.02,y+ch*.87]], dtype=np.int32)
    cv2.fillConvexPoly(frame,top,body)
    glass = np.array([[x+cw*.23,y+ch*.09],[x+cw*.77,y+ch*.09],
                      [x+cw*.87,y+ch*.36],[x+cw*.13,y+ch*.36]], dtype=np.int32)
    cv2.fillConvexPoly(frame,glass,(45,53,59))
    for xx in (x+int(cw*.06), x+int(cw*.81)):
        cv2.rectangle(frame,(xx,y+int(ch*.56)),(xx+int(cw*.12),y+int(ch*.68)), (29,26,201), -1)
    cv2.line(frame,(x+int(cw*.06),y+int(ch*.75)),(x+int(cw*.94),y+int(ch*.75)), (58,58,59),max(1,cw//100))
    if not positive:
        return frame,None,None
    plate,plate_text=plate_image(rng, cn_font, alpha_font)
    bw=int(cw*rng.uniform(.36,.48))
    bh=int(bw/3.15)
    px=x+int(cw*.5)-bw//2 +rng.randint(-max(1,cw//45),max(1,cw//45))
    py=y+int(ch*.63)
    shear=rng.randint(-max(1,bw//12),max(1,bw//12))
    tilt=rng.randint(-max(1,bh//6),max(1,bh//6))
    dest=np.float32([[px+shear,py+tilt],[px+bw+shear,py-tilt],
                     [px+bw-shear,py+bh+tilt],[px-shear,py+bh-tilt]])
    src=np.float32([[0,0],[plate.shape[1]-1,0],
                    [plate.shape[1]-1,plate.shape[0]-1],[0,plate.shape[0]-1]])
    mat=cv2.getPerspectiveTransform(src,dest)
    warp=cv2.warpPerspective(plate,mat,(w,h))
    mask=cv2.warpPerspective(np.full(plate.shape[:2],255,np.uint8),mat,(w,h))
    frame[mask>0]=warp[mask>0]
    xs=dest[:,0];ys=dest[:,1]
    x1=max(0.,float(np.min(xs)));x2=min(float(w),float(np.max(xs)))
    y1=max(0.,float(np.min(ys)));y2=min(float(h),float(np.max(ys)))
    xc=(x1+x2)/2/w;yc=(y1+y2)/2/h;ww=(x2-x1)/w;hh=(y2-y1)/h
    if not(0<xc<1 and 0<yc<1 and 0<ww<1 and 0<hh<1):
        raise RuntimeError('Generated bbox is invalid')
    return frame,(xc,yc,ww,hh),plate_text


def photometric(frame,rng):
    alpha=rng.uniform(.68,1.30)
    beta=rng.uniform(-14,17)
    frame=cv2.convertScaleAbs(frame,alpha=alpha,beta=beta)
    if rng.random()<.42:
        k=rng.choice([3,5]);frame=cv2.GaussianBlur(frame,(k,k),rng.uniform(.25,1.7))
    if rng.random()<.22:
        noise=np.random.default_rng(rng.randint(0,2**32-1)).normal(0,rng.uniform(2,9),frame.shape)
        frame=np.clip(frame.astype(np.float32)+noise,0,255).astype(np.uint8)
    return frame


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--out',type=Path,default=Path.home()/'aic_ws/plate_yolo_work/dataset_synthetic')
    ap.add_argument('--train',type=int,default=600)
    ap.add_argument('--val',type=int,default=120)
    ap.add_argument('--negative-fraction',type=float,default=.12)
    ap.add_argument('--seed',type=int,default=20261013)
    ap.add_argument('--background-dir',type=Path,help='Optional Gazebo backgrounds WITHOUT existing plates. Do not pass unlabelled positive frames.')
    ap.add_argument('--font-cjk',type=Path)
    ap.add_argument('--font-latin',type=Path)
    args=ap.parse_args()
    if args.train<1 or args.val<1 or not 0<=args.negative_fraction<.9:
        ap.error('train/val must be >=1 and 0<=negative-fraction<.9')
    cn=str(args.font_cjk) if args.font_cjk else font_location('Noto Sans CJK SC')
    lat=str(args.font_latin) if args.font_latin else font_location('DejaVu Sans Mono:style=Bold')
    if not cn or not lat or not Path(cn).is_file() or not Path(lat).is_file():
        ap.error('Font missing. Ubuntu: sudo apt install fonts-noto-cjk fonts-dejavu-core')
    cnfont=ImageFont.truetype(cn,75)
    latfont=ImageFont.truetype(lat,65)
    backgrounds=[]
    if args.background_dir:
        if not args.background_dir.is_dir():ap.error('background-dir does not exist')
        backgrounds=[p for p in args.background_dir.rglob('*') if p.suffix.lower() in ('.png','.jpg','.jpeg')]
        print(f'WARNING: using {len(backgrounds)} supplied scene backgrounds; they must have NO already visible license plates.')
    out=args.out.expanduser().resolve()
    for kind in ('train','val'):
        for k in ('images','labels'):
            p=out/k/kind
            if p.exists() and any(p.iterdir()):
                ap.error(f'Output directory {p} is not empty. Use another --out (do not mix old data).')
            p.mkdir(parents=True,exist_ok=True)
    rng=random.Random(args.seed)
    with (out/'synthetic_ground_truth.csv').open('w',encoding='utf-8',newline='') as f:
        wr=csv.writer(f);wr.writerow(['split','image','plate_text','synthetic','negative'])
        for split,count in (('train',args.train),('val',args.val)):
            for i in range(count):
                positive = rng.random()>=args.negative_fraction
                frame,box,plate_text=synthesize(rng,640,480,cnfont,latfont,backgrounds,positive=positive)
                frame=photometric(frame,rng)
                file=f'syn_{split}_{i:05d}'
                cv2.imwrite(str(out/'images'/split/(file+'.jpg')),frame,[cv2.IMWRITE_JPEG_QUALITY,rng.randint(76,96)])
                yolo_label=('0 '+' '.join(f'{v:.6f}' for v in box)+'\n') if box else ''
                (out/'labels'/split/(file+'.txt')).write_text(yolo_label,encoding='utf-8')
                wr.writerow([split,file+'.jpg',plate_text or '',1,int(not positive)])
        (out/'plate.yaml').write_text(f'path: {out}\ntrain: images/train\nval: images/val\nnames:\n  0: license_plate\n',encoding='utf-8')
    print(f'OK dataset: {out / "plate.yaml"}')
    print(f'{args.train} train + {args.val} val (synthetic); images and YOLO labels generated in sync')
    print('IMPORTANT: synthetic mAP is NOT proof of success on real Gazebo images.')

if __name__=='__main__':main()
