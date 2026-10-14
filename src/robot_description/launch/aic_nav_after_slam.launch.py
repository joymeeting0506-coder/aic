"""Start ONLY map_server+AMCL+Nav2, using newly saved map; NOT Gazebo/SLAM.

Existing Gazebo/bridge/robot_state_publisher MUST remain running.
The supervisor first deactivates and shuts down slam_toolbox to avoid a
competing /map publisher and a competing map->odom TF broadcaster.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription,TimerAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    robot=get_package_share_directory('robot_description')
    nav=get_package_share_directory('nav2_bringup')
    cfg=os.path.join(robot,'config','nav2_params_aic_v5.yaml')
    loc=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav,'launch','localization_launch.py')),
        launch_arguments={'map':LaunchConfiguration('map_yaml'),'params_file':cfg,
                          'use_sim_time':'true','autostart':'true',
                          'use_composition':'False'}.items())
    nav_stack=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(robot,'launch','nav_core_v3_1.launch.py')),
        launch_arguments={'params_file':cfg,'use_sim_time':'true','autostart':'true'}.items())
    pose=Node(package='robot_description',executable='aic_handoff_initial_pose.py',
        name='aic_handoff_initial_pose',output='screen',parameters=[{'use_sim_time':True,
          'handoff_path':LaunchConfiguration('handoff_path'),
          'ready_path':LaunchConfiguration('ready_path')}])
    return LaunchDescription([
        DeclareLaunchArgument('map_yaml',description='FULL PATH to newly saved SLAM map'),
        DeclareLaunchArgument('handoff_path',description='Validated handoff manifest'),
        DeclareLaunchArgument('ready_path',description='AMCL readiness sentinel'),
        loc,nav_stack,TimerAction(period=3.0,actions=[pose]),
    ])
