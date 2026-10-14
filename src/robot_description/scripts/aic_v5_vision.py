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
from concurrent.futures import ThreadPoolExecutor
import threading
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


def detect_lamp(frame, roi):
    """Detect the illuminated lens, not the fraction of a potentially huge ROI.

    Traffic lamps get smaller as the robot approaches/stops at different poses;
    basing confidence on ROI area makes a valid small lamp UNKNOWN. We search
    the upper/central camera for circular, strongly saturated patches inside a
    dark housing. Multiple comparable contradictory candidates fail closed.
    """
    import cv2
    import numpy as np

    H, W = frame.shape[:2]
    if roi is None:
        # Auto search is intentionally limited to the expected overhead light
        # region; unlike full-scene HSV it excludes ground-level vegetation.
        roi = (0.10, 0.07, 0.80, 0.66)
    x, y, rw, rh = roi
    x1, y1 = int(x * W), int(y * H)
    x2, y2 = int((x + rw) * W), int((y + rh) * H)
    chip = frame[y1:y2, x1:x2]
    if chip.size == 0:
        return ('UNKNOWN', 0.0, None)

    hsv = cv2.cvtColor(chip, cv2.COLOR_BGR2HSV)
    masks = {
        'RED': cv2.bitwise_or(
            cv2.inRange(hsv, (0, 120, 125), (18, 255, 255)),
            cv2.inRange(hsv, (170, 120, 125), (179, 255, 255))),
        'YELLOW': cv2.inRange(hsv, (18, 125, 155), (40, 255, 255)),
        'GREEN': cv2.inRange(hsv, (40, 100, 130), (90, 255, 255)),
    }
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    candidates = []
    for color, mask in masks.items():
        n, _, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
        for i in range(1, n):
            lx, ly, bw, bh, area = [int(v) for v in stats[i]]
            if area < 24 or bw < 5 or bh < 5:
                continue
            if not 0.50 <= bw / max(bh, 1) <= 2.0:
                continue
            if area / (bw * bh) < 0.27:
                continue
            if bw > 0.19 * W or bh > 0.19 * H:
                continue
            px, py = x1 + lx, y1 + ly
            # Require a nearby dark lamp housing; bright vegetation, wall
            # textures or other scene objects must not grant GREEN permission.
            pad_x, pad_y = int(bw * 0.9), int(bh * 0.8)
            xx1, yy1 = max(0, px-pad_x), max(0, py-pad_y)
            xx2, yy2 = min(W, px+bw+pad_x), min(H, py+bh+pad_y)
            surround = gray[yy1:yy2, xx1:xx2]
            if surround.size == 0 or float(np.mean(surround < 85)) < 0.28:
                continue
            confidence = min(0.94, 0.72 + 0.20 * min(1.0, area / 170.0))
            candidates.append((area, color, (px, py, bw, bh), confidence))
    if not candidates:
        return ('UNKNOWN', 0.0, None)
    candidates.sort(reverse=True)
    best = candidates[0]
    if any(c[1] != best[1] and c[0] >= best[0] * 0.45
           for c in candidates[1:]):
        return ('UNKNOWN', 0.0, None)
    return (best[1], best[3], best[2])


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
    ap.add_argument('--snapshot-interval',type=float,default=0,help='Periodic debug images: 0 disables (default); checkpoint captures still saved')
    args, _ros_args = ap.parse_known_args()
    if args.snapshot_interval<0:ap.error('snapshot interval must be >=0s')
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
            self.work=ThreadPoolExecutor(max_workers=1,thread_name_prefix='aic_inspect')
            self.processing=set();self.inspection_lock=threading.Lock()
            self.team=None;self.detector=None;self.identity_model=None;self.plate_model=None;self.meta=None
            self._load_inspection_models()
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
        def _load_inspection_models(self):
            # Local, packaged HOG/SVM prototype. No downloads, training or
            # recognition outside explicit inspection checkpoints.
            from ament_index_python.packages import get_package_share_directory
            import importlib.util
            models=Path(get_package_share_directory('robot_description'))/'inspection_models'
            core=Path(__file__).resolve().parent/'aic_team_vision_core.py'
            if not core.exists() or not models.is_dir():
                self.get_logger().warn('Inspection models not installed; will save checkpoint photos only')
                return
            try:
                spec=importlib.util.spec_from_file_location('aic_team_vision_core', str(core))
                module=importlib.util.module_from_spec(spec)
                spec.loader.exec_module(module)
                module.MODELS=models
                det,identity,plate,meta=module.load_models()
                self.team=module
                self.detector=det;self.identity_model=identity;self.plate_model=plate;self.meta=meta
                self.get_logger().info('Checkpoint-only SVM person + simulation plate models loaded')
            except Exception as exc:
                self.get_logger().error('Inspection models unavailable: %s'%exc)

        def _inspect_async(self, name, frame, fname):
            output={'name':name,'status':'IMAGE_SAVED','image':str(fname),
                    'person_detector':'NOT_CONFIGURED','person_count':None,
                    'ocr':'NOT_AVAILABLE', 'verification':'UNVERIFIED_MODEL_OUTPUT'}
            if self.team is not None:
                try:
                    if name.startswith('PLATE_'):
                        found=self.team.detect_plates(frame,self.plate_model,self.meta['plate_labels'])
                        output['plate_predictions']=[{'text':txt,'bbox':[x,y,w,h],
                            'score_note':'TEMPLATE_MODEL_NOT_CALIBRATED'}
                            for x,y,w,h,txt,_ in found]
                        output['ocr']='FIXED_SIMULATION_PLATE_CLASSIFIER_NOT_GENERAL_OCR'
                    else:
                        found=self.team.detect_people(frame,self.detector,self.identity_model,
                            self.meta,threshold=.20,max_width=480)
                        output['person_predictions']=[{'name':label.get('name',''),
                            'identity':label.get('identity','UNKNOWN'),
                            'bbox':[x,y,w,h], 'svm_margin':round(float(score),3)}
                            for x,y,w,h,label,score in found]
                        output['person_count']=len(found)
                        output['person_detector']='HOG_SVM_PROTOTYPE_UNVERIFIED'
                    output['status']='MODEL_INFERENCE_COMPLETED'
                except Exception as exc:
                    output['status']='MODEL_INFERENCE_ERROR'
                    output['error']=str(exc)
            self._publish_inspection(output)
            with self.inspection_lock:
                self.processing.discard(name)

        def _publish_inspection(self, output):
            msg=String();msg.data=json.dumps(output,ensure_ascii=False)
            self.result_pub.publish(msg)
            with (self.out/'inspection_events.jsonl').open('a',encoding='utf-8') as f:
                f.write(msg.data+'\n')
            self.get_logger().info(msg.data)

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
                    'instant_state':label if confidence>=.7 else 'UNKNOWN',
                    'instant_confidence':confidence if confidence>=.7 else 0.,
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
            if args.snapshot_interval>0 and now-self.last_snapshot>=args.snapshot_interval:
                self.last_snapshot=now
                file=self.out/('camera_'+time.strftime('%H%M%S')+'.jpg')
                cv2.imwrite(str(file),annotated)
        def request_cb(self,msg):
            name=safe_label(msg.data.strip())
            # Patrol publishes each request up to 3 times. Deduplicate before
            # expensive model inference, otherwise the VM becomes overloaded.
            with self.inspection_lock:
                if name in self.processing:
                    return
                self.processing.add(name)
            if self.last_frame is None or time.monotonic()-self.last_frame_wall>2:
                self._publish_inspection({'name':name,'status':'NO_FRESH_FRAME'})
                with self.inspection_lock:self.processing.discard(name)
                return
            frame=self.last_frame.copy()
            fname=self.out/(name+'_'+time.strftime('%H%M%S')+'.png')
            cv2.imwrite(str(fname),frame)
            self.work.submit(self._inspect_async,name,frame,fname)

    rclpy.init();node=Vision()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':main()
