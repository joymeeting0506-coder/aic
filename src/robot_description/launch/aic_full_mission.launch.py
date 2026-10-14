"""Fresh ROS graph: Gazebo -> SLAM -> road coverage -> AMCL/Nav2 -> guarded V5 patrol.

DO NOT launch this if an existing Gazebo/SLAM session is already running.
Default drive=false: preview only. Patrol is an additional explicit permission.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription,TimerAction,OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def supervisor(context):
    def yes(n):return LaunchConfiguration(n).perform(context).lower()=='true'
    a=[]
    if yes('drive'):a.append('--drive')
    if yes('patrol'):a.append('--patrol')
    if yes('ack_official_crossing'):a.append('--ack-official-crossing')
    # New Gazebo+SLAM: DO NOT pre-credit the previously traversed legacy route.
    return [Node(package='robot_description',executable='aic_mission_supervisor.py',
                 output='screen',arguments=a)]


def generate_launch_description():
    share=get_package_share_directory('robot_description')
    bringup=IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(share,'launch','bringup.launch.py')),
        launch_arguments={'render_mode':'sensor_safe',
                          'start_navigation':'false',
                          'start_traffic_detector':'false',
                          'robot_scale':'1.0',
                          'headless':LaunchConfiguration('headless')}.items())
    slam=IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(share,'launch','slam.launch.py')))
    return LaunchDescription([
        DeclareLaunchArgument('drive',default_value='false'),
        DeclareLaunchArgument('patrol',default_value='false'),
        DeclareLaunchArgument('ack_official_crossing',default_value='false'),
        DeclareLaunchArgument('headless',default_value='false'),
        bringup,slam,TimerAction(period=10.0,actions=[OpaqueFunction(function=supervisor)]),
    ])
