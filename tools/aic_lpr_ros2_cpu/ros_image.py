"""Decode ROS sensor_msgs/Image 8-bit camera images, including padded rows."""
import cv2
import numpy as np

def image_to_bgr(msg):
    channels={'bgr8':3,'rgb8':3,'mono8':1,'bgra8':4,'rgba8':4}
    encoding=msg.encoding.lower()
    if encoding not in channels:
        raise ValueError(f'Unsupported camera encoding {msg.encoding!r}; use an RGB camera with bgr8/rgb8/mono8/bgra8/rgba8.')
    h,w=int(msg.height),int(msg.width); n=channels[encoding];step=int(msg.step)
    if h<=0 or w<=0 or step<w*n:
        raise ValueError('Invalid image dimensions or row stride')
    raw=np.frombuffer(msg.data,dtype=np.uint8)
    if raw.size<h*step:
        raise ValueError('Image data is shorter than height * step')
    pixels=raw[:h*step].reshape(h,step)[:,:w*n].reshape(h,w,n).copy()
    if encoding=='bgr8':return pixels
    code={'rgb8':cv2.COLOR_RGB2BGR,'mono8':cv2.COLOR_GRAY2BGR,'bgra8':cv2.COLOR_BGRA2BGR,'rgba8':cv2.COLOR_RGBA2BGR}[encoding]
    return cv2.cvtColor(pixels[:,:,0] if n==1 else pixels,code)
