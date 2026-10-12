#!/usr/bin/env python3
"""frame_id 重映射节点。

gz-sim 传感器数据 frame_id 带 patrol_bot/ 前缀且用 sensor 名（如
patrol_bot/base_footprint/lidar），而 robot_state_publisher 发布的 TF 用无前缀的
link 名（如 lidar_link）。本节点订阅原始话题，重映射 frame_id 后发布到 *_clean
话题，供 SLAM / Nav2 使用。

注意：前缀里的 base_footprint 是 gz 侧合并后的名字 —— `gz sdf -p` 会把所有固定
关节的子 link 合并进 URDF 的根 link，根换了这里也要跟着换。TF 树最终是
odom -> base_footprint -> base_link -> {lidar_link, camera_link, imu_link}。

VMware SVGA 虚拟 GPU 下 gpu_lidar 的 FBO 深度渲染约 2/3 帧失败（全 inf），
本节点直接丢弃这些空帧，只转发有效帧。
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan, Image, CameraInfo, Imu
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped

FRAME_MAP = {
    'patrol_bot/base_footprint/lidar': 'lidar_link',
    'patrol_bot/base_footprint/camera': 'camera_link',
    'patrol_bot/base_footprint/imu': 'imu_link',
    # odom 的 child 就是底盘根节点 base_footprint，本身无前缀可去
    'patrol_bot/base_footprint': 'base_footprint',
    'patrol_bot/odom': 'odom',
}

# 一帧 scan 至少要有多少个有效回波才算渲染成功
# 场地四周有外墙（量程 12m 内），正常帧有效 beam 数以百计
MIN_VALID_BEAMS = 60


def remap(f):
    return FRAME_MAP.get(f, f)


class FrameRemap(Node):
    def __init__(self):
        super().__init__('frame_remap')
        sens = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )

        self.scan_pub = self.create_publisher(LaserScan, '/scan_clean', sens)
        self.odom_pub = self.create_publisher(Odometry, '/odom_clean', 10)
        self.image_pub = self.create_publisher(Image, '/camera/image_clean', sens)
        self.info_pub = self.create_publisher(CameraInfo, '/camera/camera_info_clean', sens)
        self.imu_pub = self.create_publisher(Imu, '/imu/data_clean', sens)
        self.tf_pub = self.create_publisher(TFMessage, '/tf', 10)

        self.create_subscription(LaserScan, '/scan', self.scan_cb, sens)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(Image, '/camera/image_raw', self.image_cb, sens)
        self.create_subscription(CameraInfo, '/camera/camera_info', self.info_cb, sens)
        self.create_subscription(Imu, '/imu/data', self.imu_cb, sens)
        self.dropped = 0
        self.passed = 0
        self.get_logger().info('frame_remap 已启动')

    def scan_cb(self, m):
        valid = sum(1 for r in m.ranges
                    if r == r and m.range_min <= r <= m.range_max)
        if valid < MIN_VALID_BEAMS:
            self.dropped += 1
            if self.dropped % 300 == 0:
                self.get_logger().warn(
                    f'已丢弃 {self.dropped} 个空雷达帧')
            return
        self.passed += 1
        m.header.frame_id = remap(m.header.frame_id)
        self.scan_pub.publish(m)

    def odom_cb(self, m):
        m.header.frame_id = remap(m.header.frame_id)
        m.child_frame_id = remap(m.child_frame_id)
        self.odom_pub.publish(m)

        # 从 odom 生成 odom -> base_link 的动态 TF（gz 不持续发布 TF，这里自己补）
        t = TransformStamped()
        t.header.stamp = m.header.stamp  # 用 odom 原始时间戳，与 lidar 一致
        t.header.frame_id = m.header.frame_id
        t.child_frame_id = m.child_frame_id
        t.transform.translation.x = m.pose.pose.position.x
        t.transform.translation.y = m.pose.pose.position.y
        t.transform.translation.z = m.pose.pose.position.z
        t.transform.rotation = m.pose.pose.orientation
        tfm = TFMessage()
        tfm.transforms.append(t)
        self.tf_pub.publish(tfm)

    def image_cb(self, m):
        m.header.frame_id = remap(m.header.frame_id)
        self.image_pub.publish(m)

    def info_cb(self, m):
        m.header.frame_id = remap(m.header.frame_id)
        self.info_pub.publish(m)

    def imu_cb(self, m):
        m.header.frame_id = remap(m.header.frame_id)
        self.imu_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = FrameRemap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
