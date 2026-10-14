#!/usr/bin/env python3
import sys
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

label = sys.argv[1] if len(sys.argv) > 1 else "PLATE"
out = Path.home() / "aic_ws" / "plate_dataset_raw"
out.mkdir(parents=True, exist_ok=True)

rclpy.init()
node = Node("aic_plate_image_capture")
received = [None]

def callback(msg):
    if received[0] is not None:
        return

    enc = msg.encoding.lower()
    channels = {
        "rgb8": 3, "bgr8": 3,
        "rgba8": 4, "bgra8": 4,
        "mono8": 1
    }

    if enc not in channels:
        print("不支持的编码:", enc)
        return

    c = channels[enc]
    h, w = int(msg.height), int(msg.width)
    step = int(msg.step)

    if step < w * c:
        return

    raw = np.frombuffer(msg.data, dtype=np.uint8)
    if raw.size < h * step:
        return

    pixels = raw[:h * step].reshape(h, step)
    pixels = pixels[:, :w * c].copy()

    if c == 1:
        img = pixels.reshape(h, w)
    else:
        img = pixels.reshape(h, w, c)

    if enc == "rgb8":
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    elif enc == "rgba8":
        img = cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
    elif enc == "bgra8":
        img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)

    filename = f"{label}_{datetime.now():%Y%m%d_%H%M%S_%f}.jpg"
    path = out / filename

    if not cv2.imwrite(str(path), img):
        raise RuntimeError("保存图片失败")

    received[0] = path
    print("保存成功:", path)
    print("图像尺寸:", w, "x", h)

node.create_subscription(
    Image, "/camera/image_raw",
    callback, qos_profile_sensor_data
)

print("等待真实 Gazebo 相机画面...")

deadline = time.monotonic() + 20

try:
    while received[0] is None and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.5)
    if received[0] is None:
        print("20 秒内没有收到可用画面，请检查相机话题")
finally:
    node.destroy_node()
    rclpy.shutdown()
