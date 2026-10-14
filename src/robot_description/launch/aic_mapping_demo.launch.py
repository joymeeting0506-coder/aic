"""Single command standalone Gazebo SIM + SLAM + coverage demo.

Run only in a fresh ROS graph (first close any prior Gazebo/SLAM/Nav2).
This is a mapping demo, NOT yet a competition-complete navigation mission.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription,DeclareLaunchArgument,TimerAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    d=get_package_share_directory('robot_description')
    cfg=os.path.join(d,'config','aic_sim_road_graph.json')
    bringup=IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(d,'launch','bringup.launch.py')),
        launch_arguments={'render_mode':'sensor_safe','headless':LaunchConfiguration('headless')}.items())
    slam=IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(d,'launch','slam.launch.py')))
    control=Node(package='robot_description',executable='aic_coverage_mapping.py',
                 output='screen',parameters=[{
                     'use_sim_time':True,
                     'graph_path':cfg,
                     'drive':ParameterValue(LaunchConfiguration('drive'),value_type=bool),
                     'resume_legacy_route':False}])
    return LaunchDescription([
        DeclareLaunchArgument('drive',default_value='false'),
        DeclareLaunchArgument('headless',default_value='false'),
        bringup,slam,TimerAction(period=8.0,actions=[control]),
    ])
