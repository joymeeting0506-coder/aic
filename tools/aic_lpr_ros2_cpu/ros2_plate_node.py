#!/usr/bin/env python3
"""Standalone ROS 2 camera subscriber. Publishes recognition JSON, no motion commands."""
import json
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
import torch

from common import ROOT
from predict import PlateReader
from ros_image import image_to_bgr

class PlateNode(Node):
    def __init__(self):
        super().__init__('aic_plate_reader')
        defaults=dict(image_topic='/camera/image_raw',result_topic='/plate_recognition/result',
                      detector=str(ROOT/'models/plate_detector.pt'),characters=str(ROOT/'models/characters.pt'),
                      device='cpu',rate_hz=2.0,cpu_threads=2,camera_timeout_sec=3.0)
        for key,value in defaults.items():self.declare_parameter(key,value)
        get=lambda key:self.get_parameter(key).value
        rate=float(get('rate_hz'));self.timeout=float(get('camera_timeout_sec'))
        if rate<=0 or self.timeout<=0 or int(get('cpu_threads'))<1:
            raise ValueError('rate_hz, camera_timeout_sec and cpu_threads must be positive')
        for key in ['detector','characters']:
            if not Path(get(key)).is_file():raise FileNotFoundError(get(key))
        torch.set_num_threads(int(get('cpu_threads')))
        self.reader=PlateReader(get('detector'),get('characters'),get('device'))
        self.publisher=self.create_publisher(String,get('result_topic'),10)
        qos=QoSProfile(history=HistoryPolicy.KEEP_LAST,depth=1,
                       reliability=ReliabilityPolicy.BEST_EFFORT,durability=DurabilityPolicy.VOLATILE)
        self.subscription=self.create_subscription(Image,get('image_topic'),self.on_image,qos)
        self.latest=None;self.received=None;self.warned_at=0.;self.last_text=None
        # A single-threaded executor ensures only one inference runs at a time.
        self.timer=self.create_timer(1./rate,self.on_timer)
        self.get_logger().info(f'Ready: {get("image_topic")} -> {get("result_topic")}; device={get("device")}, maximum rate={rate} Hz')

    def on_image(self,msg):
        self.latest=msg;self.received=time.monotonic()

    def publish(self,payload):
        msg=String();msg.data=json.dumps(payload,ensure_ascii=False);self.publisher.publish(msg)

    def on_timer(self):
        now=time.monotonic()
        if self.latest is None:
            if (self.received is None or now-self.received>self.timeout) and now-self.warned_at>5.:
                self.warned_at=now
                self.publish(dict(status='waiting_for_camera',plates=[]))
                self.get_logger().warning('No fresh camera image. Check topic name, Image type, ROS_DOMAIN_ID and Gazebo camera/bridge.')
            return
        msg=self.latest;self.latest=None
        header=dict(frame_id=msg.header.frame_id,stamp=dict(sec=msg.header.stamp.sec,nanosec=msg.header.stamp.nanosec))
        start=time.monotonic()
        try:
            frame=image_to_bgr(msg)
            plates=self.reader(frame)
        except Exception as exc:
            self.get_logger().error(f'Inference failed: {exc}')
            self.publish(dict(status='error',header=header,error=str(exc),plates=[]))
            return
        elapsed=(time.monotonic()-start)*1000
        self.publish(dict(status='ok',header=header,inference_ms=round(elapsed,1),plates=plates))
        summary=', '.join(p['text']+(' [review]' if p['needs_review'] else '') for p in plates) or '(no plate)'
        if summary!=self.last_text:
            self.last_text=summary;self.get_logger().info(f'{summary}; inference={elapsed:.0f} ms')

def main():
    rclpy.init();node=None
    try:
        node=PlateNode();rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        if node is not None:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':main()
