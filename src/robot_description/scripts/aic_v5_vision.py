#!/usr/bin/env python3
"""Camera-only perception scaffold for AIC 2026, fail-closed traffic gate.

Requires python3-opencv and python3-numpy in Ubuntu ROS Python environment.
Top/mid traffic ROIs MUST be calibrated from actual camera frames:
  --top-roi x,y,w,h --mid-roi x,y,w,h     (normalized fractions 0..1)
Without an ROI the published state is UNKNOWN, never GREEN.
A valid GREEN is *camera pixel evidence* only, not a traffic-rules certificate.

Optional models: --yolo-weights /path/to/local/weights.pt for person boxes;
optional pytesseract for partial ASCII OCR (Chinese plates not assured).
"""
import argparse
from collections import deque
import json
import math
import re
from pathlib import Path
import sys
import time


def validate_roi(s):
    if not s:return None
    vals=[float(x) for x in s.split(',')]
    if len(vals)!=4 or any(not math.isfinite(v) or not (0<=v<=1) for v in vals):
        raise argparse.ArgumentTypeError('ROI must be x,y,w,h normalized to [0,1]')
    x,y,w,h=vals
    if w<.02 or h<.02 or x+w>1.0001 or y+h>1.0001:
        raise argparse.ArgumentTypeError('ROI outside image or too small')
    return vals


def detect_lamp(frame,roi):
    """Conservative HSV color-blob baseline. No inference from scene state."""
    import cv2
    import numpy as np
    if roi is None:return ('UNKNOWN',0.,None)
    H,W=frame.shape[:2]
    x,y,w,h=roi; x1=int(x*W);y1=int(y*H);x2=int((x+w)*W);y2=int((y+h)*H)
    chip=frame[y1:y2,x1:x2]
    if chip.size==0:return ('UNKNOWN',0.,None)
    hsv=cv2.cvtColor(chip,cv2.COLOR_BGR2HSV)
    # Hue red wraps; white / grayed out lamps excluded by S,V floors.
    masks={
        'RED':cv2.bitwise_or(cv2.inRange(hsv,(0,125,145),(11,255,255)),cv2.inRange(hsv,(170,125,145),(179,255,255))),
        'YELLOW':cv2.inRange(hsv,(18,120,155),(40,255,255)),
        'GREEN':cv2.inRange(hsv,(40,100,130),(90,255,255)),
    }
    blobs=[]
    for color,mask in masks.items():
        n,lab,stats,center=cv2.connectedComponentsWithStats(mask,8)
        for i in range(1,n):
            cx,cy,bw,bh,area=[int(v) for v in stats[i]]
            if area<16 or bw<3 or bh<3:continue
            if not .4<=bw/max(bh,1)<=2.4:continue
            fill=area/(bw*bh)
            if fill<.28:continue
            # No need to choose a tiny bright artifact when another color is bright.
            blobs.append((area,color,(x1+cx,y1+cy,bw,bh)))
    if not blobs:return ('UNKNOWN',0.,None)
    blobs.sort(reverse=True)
    best=blobs[0]
    if len(blobs)>1 and blobs[1][1]!=best[1] and blobs[1][0]>=best[0]*.45:
        return ('UNKNOWN',0.,None)
    frac=best[0]/max(chip.shape[0]*chip.shape[1],1)
    if not (.002<=frac<=.40):return ('UNKNOWN',0.,None)
    confidence=min(.95,.60+0.25*min(frac/.035,1))
    return best[1],confidence,best[2]


def stable_light(labels):
    """A GREEN permission requires *four consecutive* matching camera frames."""
    seq=list(labels)
    return seq[-1] if len(seq)>=4 and seq[-4:] == [seq[-1]]*4 and seq[-1] in ('RED','YELLOW','GREEN') else 'UNKNOWN'


def safe_label(s):
    return re.sub(r'[^A-Za-z0-9_-]', '_', s)[:48] or 'capture'


def image_to_bgr(msg):
    import cv2
    import numpy as np
    enc=msg.encoding.lower();height=int(msg.height);width=int(msg.width)
    channels={'bgr8':3,'rgb8':3,'bgra8':4,'rgba8':4,'mono8':1}
    if enc not in channels:raise ValueError('unsupported camera encoding '+enc)
    chan=channels[enc];expected=width*chan
    if msg.step<expected:raise ValueError('camera image step too short')
    arr=np.frombuffer(bytes(msg.data),dtype=np.uint8)
    if arr.size<int(msg.step)*height:raise ValueError('camera buffer too short')
    a=arr[:int(msg.step)*height].reshape(height,int(msg.step))[:,:expected]
    img=a.reshape(height,width,chan) if chan>1 else a.reshape(height,width)
    if enc=='rgb8':return cv2.cvtColor(img,cv2.COLOR_RGB2BGR)
    if enc=='rgba8':return cv2.cvtColor(img,cv2.COLOR_RGBA2BGR)
    if enc=='bgra8':return cv2.cvtColor(img,cv2.COLOR_BGRA2BGR)
    if enc=='mono8':return cv2.cvtColor(img,cv2.COLOR_GRAY2BGR)
    return img.copy()


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--top-roi',type=validate_roi,default=None)
    ap.add_argument('--mid-roi',type=validate_roi,default=None)
    ap.add_argument('--yolo-weights',default='',help='Optional already-downloaded model file. Never silently downloads.')
    ap.add_argument('--snapshot-interval',type=float,default=5,help='Save one camera frame every N seconds')
    args, _ros_args = ap.parse_known_args()
    if args.snapshot_interval<1:ap.error('snapshot interval must be >=1s')
    try:import cv2,numpy
    except ImportError:
        raise SystemExit('Missing cv2/numpy in Ubuntu. Install: sudo apt install python3-opencv python3-numpy')
    from sensor_msgs.msg import Image,CompressedImage
    from std_msgs.msg import String
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.parameter import Parameter

    class Vision(Node):
        def __init__(self):
            super().__init__('aic_v5_camera_perception')
            self.set_parameters([Parameter('use_sim_time',Parameter.Type.BOOL,True)])
            self.target='top';self.rois={'top':args.top_roi,'mid':args.mid_roi}
            self.last_frame=None;self.last_frame_wall=0.;self.last_process=0;self.last_snapshot=0
            self.states=deque(maxlen=4)
            self.out=Path.home()/'aic_ws'/'aic_results'/('vision_'+time.strftime('%Y%m%d_%H%M%S'))
            self.out.mkdir(parents=True,exist_ok=True)
            self.traffic_pub=self.create_publisher(String,'/aic/traffic/observation',10)
            self.result_pub=self.create_publisher(String,'/aic/inspection/result',10)
            self.jpeg_pub=self.create_publisher(CompressedImage,'/aic/camera/annotated/compressed',10)
            self.create_subscription(String,'/aic/traffic/target',self.target_cb,10)
            self.create_subscription(String,'/aic/inspection/request',self.request_cb,10)
            self.create_subscription(Image,'/camera/image_raw',self.frame_cb,qos_profile_sensor_data)
            self.model=None
            if args.yolo_weights:
                p=Path(args.yolo_weights).expanduser()
                if not p.exists():raise RuntimeError('YOLO weights not present: '+str(p))
                try:
                    from ultralytics import YOLO
                    self.model=YOLO(str(p))
                except ImportError:raise RuntimeError('ultralytics unavailable; supply local dependency or omit --yolo-weights')
            self.get_logger().info('Camera perception ready; ROI top=%s mid=%s; files=%s' %
                                   (args.top_roi,args.mid_roi,self.out))
        def target_cb(self,msg):
            gate=msg.data.strip().lower()
            if gate in self.rois and gate!=self.target:
                self.target=gate;self.states.clear()
        def frame_cb(self,msg):
            now=time.monotonic()
            if now-self.last_process<.2:return  # 5 Hz to limit VMware CPU cost
            self.last_process=now
            try:frame=image_to_bgr(msg)
            except Exception as exc:
                self.get_logger().error('decode camera frame failed: %s'%exc)
                return
            self.last_frame=frame
            self.last_frame_wall=now
            label,confidence,bbox=detect_lamp(frame,self.rois[self.target])
            self.states.append(label if confidence>=.7 else 'UNKNOWN')
            stable=stable_light(self.states)
            result={'source':'camera','gate':self.target,'state':stable,
                    'confidence':confidence if stable!='UNKNOWN' else 0.,
                    'sim_stamp_ns':int(msg.header.stamp.sec)*1000000000+int(msg.header.stamp.nanosec)}
            m=String();m.data=json.dumps(result);self.traffic_pub.publish(m)
            annotated=frame.copy()
            roi=self.rois[self.target]
            if roi is not None:
                H,W=frame.shape[:2];x,y,w,h=roi
                cv2.rectangle(annotated,(int(x*W),int(y*H)),(int((x+w)*W),int((y+h)*H)),(255,255,255),2)
            if bbox:
                x,y,w,h=bbox;cv2.rectangle(annotated,(x,y),(x+w,y+h),(255,0,255),2)
            cv2.putText(annotated,f'{self.target}: {stable} ({confidence:.2f})',(12,35),
                        cv2.FONT_HERSHEY_SIMPLEX,.8,(255,255,255),2)
            success,buff=cv2.imencode('.jpg',annotated,[cv2.IMWRITE_JPEG_QUALITY,75])
            if success:
                pub=CompressedImage();pub.header=msg.header;pub.format='jpeg';pub.data=buff.tobytes()
                self.jpeg_pub.publish(pub)
            if now-self.last_snapshot>=args.snapshot_interval:
                self.last_snapshot=now
                file=self.out/('camera_'+time.strftime('%H%M%S')+'.jpg')
                cv2.imwrite(str(file),annotated)
        def request_cb(self,msg):
            name=safe_label(msg.data.strip());output={'name':name,'status':'NO_FRESH_FRAME'}
            if self.last_frame is not None and time.monotonic()-self.last_frame_wall<=2:
                fname=self.out/(name+'_'+time.strftime('%H%M%S')+'.png')
                cv2.imwrite(str(fname),self.last_frame)
                output={'name':name,'status':'IMAGE_SAVED','image':str(fname),
                        'person_detector':'NOT_CONFIGURED','person_count':None,
                        'ocr':'NOT_AVAILABLE'}
                if self.model is not None:
                    try:
                        r=self.model.predict(source=self.last_frame,conf=.3,imgsz=640,verbose=False)[0]
                        boxes=[{'xyxy':[round(float(v),1) for v in b.xyxy[0].tolist()],
                                'conf':round(float(b.conf[0]),3)}
                               for b in r.boxes if int(b.cls[0])==0]
                        output['people']=boxes;output['person_count']=len(boxes)
                        output['person_detector']='YOLO_VISUAL'
                    except Exception as exc:
                        output['person_detector']='YOLO_ERROR:'+str(exc)
                if name.startswith('PLATE_'):
                    try:
                        import pytesseract
                        gray=cv2.cvtColor(self.last_frame,cv2.COLOR_BGR2GRAY)
                        enlarged=cv2.resize(gray,None,fx=2,fy=2,interpolation=cv2.INTER_CUBIC)
                        txt=pytesseract.image_to_string(enlarged,config='--psm 11').strip()
                        output['ocr']=txt
                        output['ocr_note']='UNVERIFIED; whole image OCR and Chinese plate prefix may fail'
                    except ImportError:output['ocr']='NOT_CONFIGURED (optional pytesseract/Tesseract)'
                    except Exception as exc:output['ocr']='OCR_ERROR:'+str(exc)
            m=String();m.data=json.dumps(output,ensure_ascii=False)
            self.result_pub.publish(m)
            with (self.out/'inspection_events.jsonl').open('a',encoding='utf-8') as f:
                f.write(m.data+'\n')
            self.get_logger().info(m.data)

    rclpy.init();node=Vision()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':main()
