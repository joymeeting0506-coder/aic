"""Attach the autonomous coverage controller to an ALREADY RUNNING Gazebo+SLAM.

Default is safe PREVIEW, no motion. Use drive:=true explicitly after review.
The registered graph describes only the current simulated competition arena.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    cfg=os.path.join(get_package_share_directory('robot_description'),
                     'config','aic_sim_road_graph.json')
    return LaunchDescription([
        DeclareLaunchArgument('drive',default_value='false'),
        DeclareLaunchArgument('resume_legacy_route',default_value='false'),
        DeclareLaunchArgument('graph_path',default_value=cfg),
        Node(package='robot_description',executable='aic_coverage_mapping.py',
             name='aic_coverage_mapping',output='screen',
             parameters=[{'use_sim_time':True,
                          'drive':ParameterValue(LaunchConfiguration('drive'),value_type=bool),
                          'resume_legacy_route':ParameterValue(LaunchConfiguration('resume_legacy_route'),value_type=bool),
                          'graph_path':LaunchConfiguration('graph_path')}]),
    ])
