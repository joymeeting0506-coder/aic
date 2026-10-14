#!/usr/bin/env python3
"""红绿灯动态切换控制器（无闪烁版）。

用「先创建新模型、再删除旧模型」的双缓冲方式切换红绿灯的红-绿-黄状态，
避免先删后建造成的整根路灯瞬间消失。

状态循环：red -> green -> yellow -> red（红灯停 / 绿灯行 / 黄灯警告）。
"""
import math
import subprocess
import sys
import time

sys.path.insert(0, '/opt/ros/jazzy/opt/gz_msgs_vendor/lib/python')

import gz.transport
from gz.msgs10 import entity_factory_pb2, entity_pb2, boolean_pb2

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


# 红绿灯实例定义（基名 -> 位姿，与 world 文件里的 <include> 一致）
LIGHTS = {
    'traffic_light_top': {'x': 2.0, 'y': 3.86, 'z': 0.0, 'yaw': math.pi},
    'traffic_light_mid': {'x': 2.0, 'y': 1.30, 'z': 0.0, 'yaw': -math.pi / 2.0},
}

CYCLE = ['red', 'green', 'yellow']


def yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class TrafficLightController(Node):
    def __init__(self):
        super().__init__('traffic_light_controller')
        self.gz = gz.transport.Node()

        self.declare_parameter('red_duration', 10.0)
        self.declare_parameter('green_duration', 15.0)
        self.declare_parameter('yellow_duration', 3.0)
        self.durations = {
            'red': self.get_parameter('red_duration').value,
            'green': self.get_parameter('green_duration').value,
            'yellow': self.get_parameter('yellow_duration').value,
        }

        # 初始状态与 world 一致：top=green, mid=red
        self.state = {'traffic_light_top': 'green', 'traffic_light_mid': 'red'}

        # 每个灯的两个名字（双缓冲），初始用基名
        self.active_name = {n: n for n in LIGHTS}
        self.alt_name = {n: n + '_alt' for n in LIGHTS}

        time.sleep(2.0)
        self._wait_until_ready(timeout=30.0)
        time.sleep(1.0)

        self.next_switch = {}
        for name in LIGHTS:
            self.next_switch[name] = time.monotonic() + self.durations[self.state[name]]
            self.get_logger().info(
                f'{name}: 初始 {self.state[name]}，'
                f'{self.durations[self.state[name]]:.1f}s 后切到 '
                f'{CYCLE[(CYCLE.index(self.state[name]) + 1) % len(CYCLE)]}'
            )

        self.create_timer(0.2, self._tick)

    def _wait_until_ready(self, timeout):
        deadline = time.time() + timeout
        while time.time() < deadline:
            try:
                out = subprocess.run(
                    ['gz', 'model', '--list'], capture_output=True,
                    text=True, timeout=10,
                ).stdout
                if all(name in out for name in LIGHTS):
                    self.get_logger().info('红绿灯模型已就绪')
                    return True
            except Exception:
                pass
            time.sleep(1.0)
        self.get_logger().warn('红绿灯模型未在超时内就绪，仍继续运行')
        return False

    def _tick(self):
        now = time.monotonic()
        for name in LIGHTS:
            if now >= self.next_switch[name]:
                if self._switch(name, CYCLE[(CYCLE.index(self.state[name]) + 1) % len(CYCLE)]):
                    self.next_switch[name] = now + self.durations[self.state[name]]

    def _create(self, name, state, light):
        fac = entity_factory_pb2.EntityFactory()
        fac.name = name
        fac.sdf = (
            '<sdf version="1.10">'
            f'<include><uri>model://traffic_light_{state}</uri></include>'
            '</sdf>'
        )
        fac.pose.position.x = light['x']
        fac.pose.position.y = light['y']
        fac.pose.position.z = light['z']
        qx, qy, qz, qw = yaw_to_quat(light['yaw'])
        fac.pose.orientation.x = qx
        fac.pose.orientation.y = qy
        fac.pose.orientation.z = qz
        fac.pose.orientation.w = qw
        result, resp = self.gz.request(
            '/world/smart_community_semifinal_v8/create', fac,
            entity_factory_pb2.EntityFactory, boolean_pb2.Boolean, 4000,
        )
        return result and resp.data

    def _remove(self, name):
        req = entity_pb2.Entity()
        req.name = name
        req.type = entity_pb2.Entity.MODEL
        result, resp = self.gz.request(
            '/world/smart_community_semifinal_v8/remove', req,
            entity_pb2.Entity, boolean_pb2.Boolean, 4000,
        )
        return result and resp.data

    def _switch(self, base_name, new_state):
        """先建新模型再删旧模型，避免整根路灯消失。"""
        light = LIGHTS[base_name]
        active = self.active_name[base_name]
        alt = self.alt_name[base_name]

        # 1. 先创建新状态模型到 alt 名字
        if not self._create(alt, new_state, light):
            self.get_logger().warn(f'创建 {alt}({new_state}) 失败')
            return False
        # 2. 再删除旧状态模型
        if not self._remove(active):
            self.get_logger().warn(f'删除 {active} 失败')
            return False

        # 3. 翻转 active/alt
        self.active_name[base_name] = alt
        self.alt_name[base_name] = active
        self.state[base_name] = new_state
        self.get_logger().info(f'{base_name} -> {new_state}')
        return True


def main(args=None):
    rclpy.init(args=args)
    node = TrafficLightController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
