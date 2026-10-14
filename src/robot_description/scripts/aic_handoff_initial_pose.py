#!/usr/bin/env python3
"""AMCL initialization with the FINAL map-frame SLAM pose, no fixed start guess."""
import json
import math
import os
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped
from aic_handoff_core import load_manifest

QOS=QoSProfile(depth=10,reliability=ReliabilityPolicy.RELIABLE,
               durability=DurabilityPolicy.VOLATILE)
INIT_QOS=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,
               durability=DurabilityPolicy.TRANSIENT_LOCAL)


def diff(a,b):return math.atan2(math.sin(a-b),math.cos(a-b))


def heading(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))

class SeedAMCL(Node):
    def __init__(self):
        super().__init__('aic_handoff_initial_pose')
        self.declare_parameter('handoff_path','')
        self.declare_parameter('ready_path','')
        self.declare_parameter('timeout_sec',90.0)
        self.handoff_path=self.get_parameter('handoff_path').value
        self.ready_path=self.get_parameter('ready_path').value
        self.data=load_manifest(self.handoff_path)
        self.start=time.monotonic()
        self.pub=self.create_publisher(PoseWithCovarianceStamped,'/initialpose',INIT_QOS)
        self.sub=self.create_subscription(PoseWithCovarianceStamped,'/amcl_pose',self.on_pose,QOS)
        self.sent=0;self.good=0;self.done=False
        self.timer=self.create_timer(1.0,self.tick)
        self.get_logger().info('Waiting for AMCL to accept handoff pose')
    def make_pose(self):
        p=self.data['pose_map']
        msg=PoseWithCovarianceStamped()
        msg.header.frame_id='map';msg.header.stamp=self.get_clock().now().to_msg()
        msg.pose.pose.position.x=float(p['x'])
        msg.pose.pose.position.y=float(p['y'])
        msg.pose.pose.orientation.z=math.sin(float(p['yaw'])/2)
        msg.pose.pose.orientation.w=math.cos(float(p['yaw'])/2)
        cov=[0.]*36
        cov[0]=.03**2;cov[7]=.03**2;cov[35]=.10**2
        msg.pose.covariance=cov
        return msg
    def on_pose(self,msg):
        if not self.sent or self.done:return
        expected=self.data['pose_map'];actual=msg.pose.pose
        dx=math.hypot(actual.position.x-float(expected['x']),
                      actual.position.y-float(expected['y']))
        da=abs(diff(heading(actual.orientation),float(expected['yaw'])))
        if dx<.16 and da<.30:
            self.good+=1
        else:
            self.good=0
        if self.good>=3:
            self.done=True
            if self.ready_path:
                p=Path(self.ready_path);p.parent.mkdir(parents=True,exist_ok=True)
                tmp=p.with_suffix('.tmp')
                tmp.write_text(json.dumps({'ready':True,'map_yaml':self.data['map_yaml'],
                    'position_error':dx,'yaw_error':da,'clock':time.time()}),encoding='utf-8')
                os.replace(tmp,p)
            self.get_logger().info('AMCL_HANDOFF_READY')
            self.timer.cancel()
    def tick(self):
        if self.done:return
        if time.monotonic()-self.start > float(self.get_parameter('timeout_sec').value):
            self.get_logger().error('AMCL_HANDOFF_TIMEOUT')
            self.timer.cancel();return
        if self.count_subscribers('/initialpose')==0:return
        if self.get_clock().now().nanoseconds<=0:return
        self.pub.publish(self.make_pose());self.sent+=1
        if self.sent==1:self.get_logger().info('Initial pose published from the saved SLAM pose')

def main():
    rclpy.init();n=None
    try:
        n=SeedAMCL();rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:
        if n:n.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
