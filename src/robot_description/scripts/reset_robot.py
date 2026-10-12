#!/usr/bin/env python3
"""重置机器人到 spawn 位置（起点线 3.88, 3.88）朝向 -X，解决卡墙。
与 spawn_robot.launch.py 的默认值保持一致。"""
import math
import sys
import time

sys.path.insert(0, '/opt/ros/jazzy/opt/gz_msgs_vendor/lib/python')

import gz.transport
from gz.msgs10 import pose_pb2, boolean_pb2


def main():
    node = gz.transport.Node()
    time.sleep(1.0)

    req = pose_pb2.Pose()
    req.name = 'patrol_bot'
    req.position.x = 3.88
    req.position.y = 3.88
    req.position.z = 0.15
    yaw = math.pi
    req.orientation.z = math.sin(yaw / 2.0)
    req.orientation.w = math.cos(yaw / 2.0)

    result, resp = node.request(
        '/world/smart_community_semifinal_v8/set_pose', req,
        pose_pb2.Pose, boolean_pb2.Boolean, 4000,
    )
    print(f'set_pose 结果: result={result} data={resp.data}')


if __name__ == '__main__':
    main()
