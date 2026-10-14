#!/usr/bin/env python3
"""Manual single-class YOLO plate box annotation (requires graphical desktop)."""
from pathlib import Path
import argparse, cv2

ap=argparse.ArgumentParser()
ap.add_argument('--raw',type=Path,default=Path.home()/'aic_ws/plate_yolo_work/raw')
a=ap.parse_args()
images=a.raw/'images'; labels=a.raw/'labels'; labels.mkdir(parents=True,exist_ok=True)
files=sorted(p for p in images.glob('*') if p.suffix.lower() in ('.jpg','.jpeg','.png') and not (labels/(p.stem+'.txt')).exists())
print('Draw rectangle over LICENSE PLATE only (not whole car).')
print('Keys: n = save boxes and next; 0 = save NEGATIVE image with no plate; r = remove rectangles; s = skip unlabeled; q = quit')
print('WARNING: do NOT press 0 if the image does contain a visible plate.')
if not files: print('No unlabelled images.');raise SystemExit(0)
window='AIC plate annotator'
cv2.namedWindow(window,cv2.WINDOW_NORMAL)
for idx,path in enumerate(files):
    img=cv2.imread(str(path))
    if img is None:
        print('Unreadable:',path);continue
    H,W=img.shape[:2]; f=min(1.0,1100/W,800/H)
    dw,dh=max(1,round(W*f)),max(1,round(H*f))
    view=cv2.resize(img,(dw,dh))
    rectangles=[];start=None;drag=None
    def mouse(event,x,y,flags,param):
        global start,drag
        if event==cv2.EVENT_LBUTTONDOWN: start=(x,y);drag=start
        elif event==cv2.EVENT_MOUSEMOVE and start: drag=(x,y)
        elif event==cv2.EVENT_LBUTTONUP and start:
            x1,x2=sorted((start[0],x));y1,y2=sorted((start[1],y))
            if x2-x1>5 and y2-y1>5: rectangles.append((x1,y1,x2,y2))
            start=None;drag=None
    cv2.setMouseCallback(window,mouse)
    while True:
        display=view.copy()
        for x1,y1,x2,y2 in rectangles: cv2.rectangle(display,(x1,y1),(x2,y2),(0,255,0),2)
        if start and drag: cv2.rectangle(display,start,drag,(0,200,255),2)
        title=f'{idx+1}/{len(files)} {path.name[:50]} | boxes {len(rectangles)} | n save 0 negative r clear s skip q quit'
        cv2.putText(display,title,(6,22),cv2.FONT_HERSHEY_SIMPLEX,.45,(0,0,0),3)
        cv2.putText(display,title,(6,22),cv2.FONT_HERSHEY_SIMPLEX,.45,(255,255,255),1)
        cv2.imshow(window,display)
        key=cv2.waitKey(40)&0xff
        if key==ord('r'): rectangles=[]
        if key==ord('q'): cv2.destroyAllWindows();raise SystemExit(0)
        if key==ord('s'):break
        if key in (ord('n'),ord('0')):
            if key==ord('n') and not rectangles:
                print('Draw a box or press 0 to explicitly label a negative frame.');continue
            if key==ord('0') and rectangles:
                print('Press r to clear boxes before marking negative.');continue
            yolo=[]
            for x1,y1,x2,y2 in rectangles:
                x1=max(0,min(W,x1/f));x2=max(0,min(W,x2/f));y1=max(0,min(H,y1/f));y2=max(0,min(H,y2/f))
                xc=(x1+x2)/(2*W);yc=(y1+y2)/(2*H);w=(x2-x1)/W;h=(y2-y1)/H
                if 0<xc<1 and 0<yc<1 and w>0 and h>0:yolo.append(f'0 {xc:.6f} {yc:.6f} {w:.6f} {h:.6f}')
            (labels/(path.stem+'.txt')).write_text('\n'.join(yolo)+('\n' if yolo else ''),encoding='utf8')
            print('Saved',path.name,'boxes',len(yolo));break
cv2.destroyAllWindows()
