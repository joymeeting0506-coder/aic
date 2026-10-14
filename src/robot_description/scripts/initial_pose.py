#!/usr/bin/env python3
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
)


class InitialPosePublisher(Node):
    """Seed AMCL at the known SLAM START pose.

    The robot is spawned at the exact START used when the saved map was created.
    During that SLAM run map->base_footprint was approximately identity at START,
    so START corresponds to map=(0, 0, 0).

    This node does NOT rely on a fixed startup delay. It waits for:
      1) simulation time to start;
      2) at least one /initialpose subscriber (AMCL);
    and then publishes repeatedly until AMCL responds or a safe timeout expires.
    """

    def __init__(self):
        super().__init__('aic_initial_pose_publisher')

        pub_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        amcl_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', pub_qos
        )
        self.sub = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.amcl_cb, amcl_qos
        )

        self.started_wall = time.monotonic()
        self.first_publish_wall = None
        self.publish_count = 0
        self.amcl_pose_received = False
        self.done_timer = None
        self.timer = self.create_timer(0.5, self.tick)

    def amcl_cb(self, _msg):
        if self.publish_count > 0:
            self.amcl_pose_received = True

    def make_msg(self):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = 0.0
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.z = 0.0
        msg.pose.pose.orientation.w = 1.0

        # Known spawn pose, but leave enough room for AMCL to settle.
        cov = [0.0] * 36
        cov[0] = 0.04       # x variance (0.20 m sigma)
        cov[7] = 0.04       # y variance
        cov[35] = 0.0225    # yaw variance (~0.15 rad sigma)
        msg.pose.covariance = cov
        return msg

    def finish(self, reason):
        if self.timer:
            self.timer.cancel()
        self.get_logger().info(
            f'Initial pose seeding complete: {reason}; '
            f'published={self.publish_count}'
        )
        self.done_timer = self.create_timer(0.5, rclpy.shutdown)

    def tick(self):
        # With use_sim_time=true, time remains zero until /clock is received.
        if self.get_clock().now().nanoseconds == 0:
            if time.monotonic() - self.started_wall > 30.0:
                self.get_logger().error('No simulation clock after 30 s.')
                self.finish('NO_SIM_CLOCK')
            return

        if self.pub.get_subscription_count() < 1:
            if time.monotonic() - self.started_wall > 45.0:
                self.get_logger().error('AMCL did not subscribe to /initialpose.')
                self.finish('NO_AMCL_SUBSCRIBER')
            return

        if self.amcl_pose_received:
            self.finish('AMCL_POSE_RECEIVED')
            return

        self.pub.publish(self.make_msg())
        self.publish_count += 1
        if self.first_publish_wall is None:
            self.first_publish_wall = time.monotonic()

        if self.publish_count == 1 or self.publish_count % 5 == 0:
            self.get_logger().info(
                f'Published START initial pose map=(0,0,0), '
                f'attempt={self.publish_count}'
            )

        if self.first_publish_wall and time.monotonic() - self.first_publish_wall > 20.0:
            self.get_logger().warn(
                'AMCL pose was not observed, stopping repeated initial-pose seeding '
                'after 20 s. Run tools/check_nav2.sh before moving.'
            )
            self.finish('PUBLISH_TIMEOUT')


def main():
    rclpy.init()
    node = InitialPosePublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
