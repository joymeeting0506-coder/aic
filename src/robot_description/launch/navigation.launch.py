import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot_share = get_package_share_directory('robot_description')
    world_share = get_package_share_directory('smart_community_gazebo')
    nav2_share = get_package_share_directory('nav2_bringup')

    map_yaml = os.path.join(world_share, 'maps', 'community.yaml')
    nav2_params = os.path.join(robot_share, 'config', 'nav2_params_aic_v3.yaml')
    rviz_cfg = os.path.join(nav2_share, 'rviz', 'nav2_default_view.rviz')

    use_rviz = LaunchConfiguration('rviz')
    headless = LaunchConfiguration('headless')
    autostart = LaunchConfiguration('autostart')

    # Starts Gazebo, patrol_bot, ros_gz_bridge and frame_remap.
    # VMware + gpu_lidar MUST use sensor_safe: ogre was measured as 360 rays at 0.1 m.
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_share, 'launch', 'bringup.launch.py')
        ),
        launch_arguments={
            'render_mode': 'sensor_safe',
            'headless': headless,
        }.items(),
    )

    # Static-map navigation mode: map_server + AMCL + Nav2 stack.
    # No slam_toolbox is launched here.
    # The standard Jazzy bringup starts docking_server, which directly publishes
    # /cmd_vel alongside collision_monitor. This competition does not dock.
    # Launch the official localization stack once and our no-docking Nav2 core.
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share, 'launch', 'localization_launch.py')
        ),
        launch_arguments={
            'map': map_yaml,
            'params_file': nav2_params,
            'use_sim_time': 'true',
            'autostart': autostart,
            'use_composition': 'False',
        }.items(),
    )
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_share, 'launch', 'nav_core_v3_1.launch.py')
        ),
        launch_arguments={
            'params_file': nav2_params,
            'use_sim_time': 'true',
            'autostart': autostart,
        }.items(),
    )

    # Start early; the node itself waits for simulation time and an AMCL subscriber,
    # so this is robust against slower VMware startup.
    initial_pose = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='robot_description',
                executable='initial_pose.py',
                name='aic_initial_pose',
                output='screen',
                parameters=[{'use_sim_time': True}],
            )
        ],
    )

    rviz = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_cfg],
                output='screen',
                condition=IfCondition(use_rviz),
                parameters=[{'use_sim_time': True}],
            )
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz', default_value='false',
            description='Launch RViz with Nav2 default view'
        ),
        DeclareLaunchArgument(
            'headless', default_value='false',
            description='Use Gazebo server-only mode on resource-constrained VM'
        ),
        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Automatically activate Nav2 lifecycle nodes'
        ),
        gazebo,
        localization,
        nav2,
        initial_pose,
        rviz,
    ])
