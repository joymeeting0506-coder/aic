#!/usr/bin/env bash
# 终端2：激活 Nav2 生命周期 + 发布初始位姿（一条命令，避免手抄 YAML 出错）
source /home/biao/aic/install/setup.bash

act() {
  local n="$1"
  # 先直接 activate（已 configure 时成功）；失败则 configure 后再 activate
  ros2 lifecycle set "$n" activate 2>/dev/null || {
    ros2 lifecycle set "$n" configure 2>/dev/null || true
    ros2 lifecycle set "$n" activate 2>/dev/null || true
  }
}

echo "== 1) 激活定位 =="
act /map_server
act /amcl

echo "== 2) 等待 AMCL 就绪，发布初始位姿 (3.88, 3.88, yaw=pi) =="
sleep 3
python3 - <<'PY'
import rclpy, time
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped

rclpy.init()
node = Node('set_initial_pose')
pub = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

msg = PoseWithCovarianceStamped()
msg.header.frame_id = 'map'
msg.pose.pose.position.x = 3.88
msg.pose.pose.position.y = 3.88
msg.pose.pose.orientation.z = 1.0   # yaw=pi -> z=sin(pi/2)=1
msg.pose.pose.orientation.w = 0.0   #           w=cos(pi/2)=0
# 非零协方差：允许少量误差，让 AMCL 靠雷达收敛，避免一上来被锁死朝向
msg.pose.covariance[0]  = 0.25    # x 方差
msg.pose.covariance[7]  = 0.25    # y 方差
msg.pose.covariance[35] = 0.0685  # yaw 方差 (~0.26 rad)

# 发 3 次（间隔 1s），确保慢虚拟机下 AMCL 至少收到一次
for _ in range(3):
    pub.publish(msg)
    rclpy.spin_once(node, timeout_sec=0.1)
    time.sleep(1.0)

node.destroy_node()
rclpy.shutdown()
print('初始位姿已发布 3 次')
PY

echo "== 3) 激活导航 =="
act /planner_server
act /controller_server
act /smoother_server
act /behavior_server
act /bt_navigator
act /velocity_smoother
act /collision_monitor

echo "== 完成，检查 amcl 状态 =="
ros2 lifecycle get /amcl
echo ""
echo "提示：等 5 秒让 AMCL 收敛（雷达对上地图）后，再启动终端 3 的巡检。"
