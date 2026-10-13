"""Gazebo+AMCL+Nav2+camera-only perception based on tested v3.1 launch.

ROI empty by default: vision reports UNKNOWN; traffic gate remains CLOSED.
Use top_roi:=x,y,w,h after reviewing saved camera screenshots.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription,TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    robot=get_package_share_directory('robot_description')
    nav=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(robot,'launch','aic_v5_navigation.launch.py')),
        launch_arguments={'headless':LaunchConfiguration('headless'),
                          'rviz':LaunchConfiguration('rviz')}.items())
    vision=TimerAction(period=6.0,actions=[Node(
        package='robot_description',executable='aic_v5_vision.py',output='screen',
        arguments=[['--top-roi=',LaunchConfiguration('top_roi')],
                   ['--mid-roi=',LaunchConfiguration('mid_roi')],
                   ['--yolo-weights=',LaunchConfiguration('yolo_weights')]],
        condition=IfCondition(LaunchConfiguration('perception')))])
    # Optional teammate colour detection for demonstration ONLY.
    # It publishes /traffic_light/*, NOT the v5.4 /aic/traffic/observation gate.
    team_traffic_demo=TimerAction(period=7.0,actions=[Node(
        package='robot_description',executable='traffic_light_detector.py',
        name='traffic_light_detector',output='screen',
        parameters=[{'use_sim_time':True}],
        condition=IfCondition(LaunchConfiguration('team_traffic_demo')))])
    return LaunchDescription([
        DeclareLaunchArgument('headless',default_value='false'),
        DeclareLaunchArgument('rviz',default_value='false'),
        DeclareLaunchArgument('perception',default_value='true'),
        DeclareLaunchArgument('team_traffic_demo',default_value='false',
            description='Teammate red/green/yellow recognition VISUAL DEMO, no gate permission'),
        DeclareLaunchArgument('top_roi',default_value=''),
        DeclareLaunchArgument('mid_roi',default_value=''),
        DeclareLaunchArgument('yolo_weights',default_value='',
            description='Optional local YOLO weights; empty means NOT_CONFIGURED'),
        nav,vision,team_traffic_demo])
