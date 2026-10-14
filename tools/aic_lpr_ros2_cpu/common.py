"""Shared paths, alphabet and geometric operations for the AIC baseline."""
from pathlib import Path
import cv2
import numpy as np

ROOT = Path(__file__).resolve().parent
PROVINCES = '京津沪渝冀晋蒙辽吉黑苏浙皖闽赣鲁豫鄂湘粤桂琼川贵云藏陕甘青宁新'
LETTERS = 'ABCDEFGHJKLMNPQRSTUVWXYZ'
ALNUM = LETTERS + '0123456789'
CHARS = PROVINCES + ALNUM
ANNOTATIONS = [
    {'file': 'scene_1.png', 'quad': [[270,338],[381,338],[382,373],[269,373]], 'text': '皖BK3P58', 'group': 'train'},
    {'file': 'scene_2.png', 'quad': [[282,346],[383,347],[384,380],[281,379]], 'text': '皖BK3P58', 'group': 'train'},
    {'file': 'scene_3.png', 'quad': [[36,340],[166,341],[164,377],[34,376]], 'text': '皖BK3P58', 'group': 'held_out_scene'},
    {'file': 'scene_4.png', 'quad': [[98,98],[173,98],[174,122],[97,122]], 'text': '苏APL12A', 'group': 'train'},
]

def read_image(path):
    im = cv2.imdecode(np.fromfile(str(path), dtype=np.uint8), cv2.IMREAD_COLOR)
    if im is None:
        raise ValueError(f'Cannot read image: {path}')
    return im

def transform(points, matrix):
    return cv2.perspectiveTransform(np.asarray(points, np.float32).reshape(1,-1,2), matrix)[0]

def corners(w, h):
    return np.float32([[0,0],[w-1,0],[w-1,h-1],[0,h-1]])

def rect_points(box):
    x1,y1,x2,y2 = box
    return np.float32([[x1,y1],[x2,y1],[x2,y2],[x1,y2]])

def bbox(points):
    p = np.asarray(points)
    return [float(p[:,0].min()), float(p[:,1].min()), float(p[:,0].max()), float(p[:,1].max())]

def crop_plate(frame, box, padding=.07):
    x1,y1,x2,y2 = box
    dx,dy=(x2-x1)*padding,(y2-y1)*padding
    h,w=frame.shape[:2]
    xy=[max(0,int(x1-dx)),max(0,int(y1-dy)),min(w,int(np.ceil(x2+dx))),min(h,int(np.ceil(y2+dy)))]
    return frame[xy[1]:xy[3],xy[0]:xy[2]], xy
