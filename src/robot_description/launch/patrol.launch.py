"""航点巡检启动文件（视觉识别 + 巡检调度）。

前置条件：
  1. 已启动仿真 + 导航（bringup.launch.py 或 nav_bringup.launch.py，二者都含
     Gazebo / 红绿灯 / 红绿灯识别 / localization / navigation）
  2. Nav2 lifecycle 节点已全部激活（map_server / amcl / planner_server /
     bt_navigator / behavior_server / velocity_smoother / smoother_server）
  3. 已发布 /initialpose 设置机器人初始位姿

本文件拉起：人偶+车牌视觉识别节点、航点巡检调度节点。

用法：
  ros2 launch robot_description patrol.launch.py
  ros2 launch robot_description patrol.launch.py loop:=true
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    loop = LaunchConfiguration('loop')
    vision_dir = LaunchConfiguration('vision_dir')

    return LaunchDescription([
        DeclareLaunchArgument('loop', default_value='false',
                              description='是否循环巡检'),
        DeclareLaunchArgument('vision_dir', default_value='',
                              description='vision_training 目录（空则按脚本路径自动解析）'),

        # 人偶身份识别 + 车牌 OCR
        Node(
            package='robot_description',
            executable='vision_detector.py',
            name='vision_detector',
            output='screen',
            parameters=[{'vision_dir': vision_dir}],
        ),

        # 航点巡检调度
        Node(
            package='robot_description',
            executable='waypoint_patrol.py',
            name='waypoint_patrol',
            output='screen',
            parameters=[{'loop': loop}],
        ),
    ])
