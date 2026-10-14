#!/usr/bin/env python3
"""航点巡检调度节点。

读取 official_route.yaml 里的航点序列，逐点调用 Nav2 的 /navigate_to_pose
action 导航到目标；到达后根据该航点的 action 标签做不同处理：

  - traffic_light_wait : 停车等待，订阅 /traffic_light/state，直到绿灯再走
  - view_*             : 触发 /vision/trigger 做人偶身份识别并打印结果
  - read_plate_*       : 触发 /vision/trigger 做车牌 OCR 并打印结果
  - depart_left / continue_left / turn_* / turn_finish : 直接通过，不停留
  - finish             : 汇总街区统计（社区/非社区人数），巡检结束

用法（在 aic_ws 下，Nav2 已启动并激活后）：
  ros2 run robot_description waypoint_patrol.py
  ros2 run robot_description waypoint_patrol.py --loop   # 循环巡检
  ros2 run robot_description waypoint_patrol.py --route /绝对路径/official_route.yaml
"""
import json
import math
import os
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String

try:
    import yaml
except ImportError:
    print('[waypoint_patrol] 需要 PyYAML：pip install pyyaml 或 sudo apt install python3-yaml',
          file=sys.stderr)
    raise

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from aic_v5_route import to_local


# 普通通过点轻微停顿（让导航控制器收敛）
DEFAULT_PASS_SEC = 0.5


class WaypointPatrol(Node):
    def __init__(self):
        super().__init__('waypoint_patrol')

        # ---- 参数 ----
        self.declare_parameter('loop', False)
        self.declare_parameter('route_file', '')  # 空则自动找 smart_community_gazebo 包
        self.declare_parameter('nav_action_timeout', 120.0)  # 单点导航超时（秒）
        self.declare_parameter('light_wait_timeout', 60.0)    # 红灯最长等待
        self.declare_parameter('vision_timeout', 30.0)   # 单次视觉识别最长等待（秒）

        self.loop = self.get_parameter('loop').value
        self.route_file = self.get_parameter('route_file').value
        self.nav_timeout = self.get_parameter('nav_action_timeout').value
        self.light_timeout = self.get_parameter('light_wait_timeout').value
        self.vision_timeout = self.get_parameter('vision_timeout').value

        # ---- 红绿灯状态订阅 ----
        self.light_state = 'UNKNOWN'
        self._green_seen = False   # 已识别到绿灯后，跳过后续红绿灯等待（第二个灯识别不稳定）
        self.create_subscription(String, '/traffic_light/state', self._light_cb, 10)

        # ---- 视觉识别触发 / 结果订阅 ----
        self.vision_trigger_pub = self.create_publisher(String, '/vision/trigger', 10)
        self._vision_results = {}   # id -> 解析后的结果 dict
        self._trigger_seq = 0
        self.create_subscription(String, '/vision/result', self._vision_result_cb, 10)

        # ---- Nav2 action 客户端 ----
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.get_logger().info('等待 /navigate_to_pose action 服务...')
        if not self.nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('/navigate_to_pose 不可用！请先启动 Nav2 并激活 lifecycle 节点')
            raise RuntimeError('nav2 action server not available')
        self.get_logger().info('Nav2 action 已连接')

        # ---- 加载航点 ----
        self.route = self._load_route()
        self.get_logger().info(f'加载航点 {len(self.route)} 个：'
                               f'{" -> ".join(w["name"] for w in self.route)}')

    # ------------------------------------------------------------------
    def _light_cb(self, msg: String):
        self.light_state = msg.data

    def _vision_result_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        self._vision_results[data.get('id')] = data

    def _load_route(self):
        if not self.route_file:
            # 自动从 smart_community_gazebo 包 share 目录找
            try:
                from ament_index_python.packages import get_package_share_directory
                share = get_package_share_directory('smart_community_gazebo')
                self.route_file = os.path.join(share, 'config', 'official_route.yaml')
            except Exception as e:
                self.get_logger().error(f'无法定位 route 文件：{e}')
                raise
        self.get_logger().info(f'航点文件：{self.route_file}')
        with open(self.route_file, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f)
        route = data.get('route', [])
        if not route or route[0].get('name') != 'START':
            raise ValueError('Route must begin at START for the selected SLAM map')
        # The retained SLAM map was built at the V5 official START. The team
        # route's START differs by 2.4 cm and must not redefine the map frame.
        from ament_index_python.packages import get_package_share_directory
        reference_file = os.path.join(
            get_package_share_directory('smart_community_gazebo'),
            'config', 'official_route.yaml')
        with open(reference_file, 'r', encoding='utf-8') as f:
            reference = yaml.safe_load(f)['route'][0]
        if reference.get('name') != 'START':
            raise ValueError('Map reference route must begin at START')
        return [dict(wp, **to_local(wp, reference)) for wp in route]

    # ------------------------------------------------------------------
    @staticmethod
    def _yaw_to_quat(yaw):
        return {'z': math.sin(yaw / 2.0), 'w': math.cos(yaw / 2.0)}

    def _send_goal(self, wp):
        """导航到单个航点，返回 True/False。"""
        goal = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(wp['x'])
        pose.pose.position.y = float(wp['y'])
        q = self._yaw_to_quat(float(wp['yaw']))
        pose.pose.orientation.z = q['z']
        pose.pose.orientation.w = q['w']
        goal.pose = pose

        self.get_logger().info(f'>>> 导航到 {wp["name"]}  '
                               f'({wp["x"]:.2f}, {wp["y"]:.2f}, yaw={wp["yaw"]:.2f})  '
                               f'action={wp.get("action", "")}')

        send_future = self.nav_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)
        goal_handle = send_future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error(f'    {wp["name"]} 目标被拒绝！')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future,
                                         timeout_sec=self.nav_timeout)
        if not result_future.done():
            self.get_logger().warn(f'    {wp["name"]} 导航超时，取消目标')
            goal_handle.cancel_goal_async()
            return False

        status = result_future.result().status
        if status == 4:  # GoalStatus.STATUS_SUCCEEDED
            self.get_logger().info(f'    ✔ 到达 {wp["name"]}')
            return True
        else:
            self.get_logger().warn(f'    ✘ {wp["name"]} 导航未成功（status={status}）')
            return False

    # ------------------------------------------------------------------
    def _do_waypoint_action(self, wp):
        """到达航点后，根据 action 标签做停留/等待或触发视觉识别。"""
        action = wp.get('action', '')
        name = wp['name']

        if action == 'traffic_light_wait':
            self._wait_for_green(name)
        elif action.startswith('view'):
            self._do_person_recognition(name)
        elif action.startswith('read_plate'):
            self._do_plate_recognition(name)
        elif action == 'finish':
            self._do_final_stats()
        else:
            # depart_left / continue_left / turn_* / turn_finish 等通过点
            self._sleep(DEFAULT_PASS_SEC)

    def _trigger_vision(self, mode, timeout=None):
        """发送视觉触发，等待匹配 id 的结果，返回结果 dict 或 None。"""
        if timeout is None:
            timeout = self.vision_timeout
        self._trigger_seq += 1
        seq = self._trigger_seq
        self._vision_results.pop(seq, None)
        self.vision_trigger_pub.publish(
            String(data=json.dumps({'id': seq, 'mode': mode})))
        deadline = time.time() + timeout
        while rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            res = self._vision_results.pop(seq, None)
            if res is not None:
                return res
            time.sleep(0.02)
        self.get_logger().warn(f'等待视觉结果（{mode}）超时 {timeout:.0f}s')
        return None

    def _do_person_recognition(self, wp_name):
        self.get_logger().info(f'    [人偶识别] {wp_name} 触发视觉识别...')
        res = self._trigger_vision('person')
        people = (res or {}).get('people', [])
        if not people:
            self.get_logger().warn(f'    [人偶识别] {wp_name} 未识别到人偶')
            return
        summary = []
        for p in people:
            ident = p.get('identity', '')
            zh = ('社区人员' if ident == 'community'
                  else ('非社区人员' if ident == 'noncommunity' else ident))
            summary.append(f'{zh}({p.get("name", "")})')
            self.get_logger().info(
                f'    [人偶] 类别=人偶 内容={zh}({p.get("name", "")}) '
                f'置信度={p.get("confidence", 0):.2f}')
        self.get_logger().info(f'    [人偶识别] {wp_name} 完成：{"、".join(summary)}')

    def _do_plate_recognition(self, wp_name):
        self.get_logger().info(f'    [车牌识别] {wp_name} 触发视觉识别...')
        res = self._trigger_vision('plate')
        plates = (res or {}).get('plates', [])
        if not plates:
            self.get_logger().warn(f'    [车牌识别] {wp_name} 未识别到车牌')
            return
        for p in plates:
            self.get_logger().info(
                f'    [车牌] 类别=车牌 内容={p.get("text", "")} '
                f'置信度={p.get("confidence", 0):.2f}')

    def _do_final_stats(self):
        self.get_logger().info('=== 巡检完成，汇总街区统计 ===')
        res = self._trigger_vision('stats')
        stats = (res or {}).get('stats', {})
        if not stats:
            self.get_logger().warn('未取得街区统计')
            return
        self.get_logger().info(
            f'    [街区统计] 社区人员={stats.get("community", 0)} '
            f'非社区人员={stats.get("noncommunity", 0)} '
            f'共识别={stats.get("total_seen", 0)}')

    def _wait_for_green(self, wp_name):
        """在红绿灯等待点停车，直到绿灯（或超时）。"""
        if self._green_seen:
            self.get_logger().info(f'    [红灯等待] {wp_name} 已识别过绿灯，跳过本次等待')
            return
        self.get_logger().info(f'    [红灯等待] {wp_name} 当前灯色={self.light_state}，'
                               f'最长等待 {self.light_timeout}s')
        start = time.time()
        warned_yellow = False
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            elapsed = time.time() - start
            state = self.light_state

            if state == 'GREEN':
                self._green_seen = True
                self.get_logger().info(f'    🟢 绿灯！继续前进（等了 {elapsed:.1f}s）')
                return
            if state == 'YELLOW' and not warned_yellow:
                self.get_logger().info(f'    🟡 黄灯，继续等待变绿...')
                warned_yellow = True
            if elapsed > self.light_timeout:
                self.get_logger().warn(f'    ⚠ 等待绿灯超时 {self.light_timeout}s，强制通行')
                return
            time.sleep(0.1)

    def _sleep(self, sec):
        end = time.time() + sec
        while rclpy.ok() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.05)

    # ------------------------------------------------------------------
    def run(self):
        round_no = 0
        while rclpy.ok():
            round_no += 1
            self.get_logger().info(f'========== 第 {round_no} 轮巡检开始 ==========')
            for wp in self.route:
                if not rclpy.ok():
                    break
                ok = self._send_goal(wp)
                if not ok:
                    self.get_logger().warn(f'航点 {wp["name"]} 失败，跳过继续下一点')
                    # 也可选择 break 停下来人工介入
                self._do_waypoint_action(wp)
                if wp.get('action') == 'finish':
                    break

            if not self.loop:
                break
            self.get_logger().info('--- 循环模式：5s 后开始下一轮 ---')
            self._sleep(5.0)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = WaypointPatrol()
        node.run()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        if node:
            node.get_logger().error(f'巡检节点异常：{e}')
        else:
            print(f'初始化失败：{e}', file=sys.stderr)
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
