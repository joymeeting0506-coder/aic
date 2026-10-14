import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    world_pkg = get_package_share_directory('smart_community_gazebo')
    robot_pkg = get_package_share_directory('robot_description')
    gz_pkg = get_package_share_directory('ros_gz_sim')
    slam_pkg = get_package_share_directory('slam_toolbox')

    world = os.path.join(world_pkg, 'worlds', 'smart_community_v2.sdf')
    models = os.path.join(world_pkg, 'models')
    slam_params = os.path.join(robot_pkg, 'config', 'mapper_params_online_async.yaml')

    render_mode = LaunchConfiguration('render_mode')
    software_render = PythonExpression([
        "'", render_mode, "' in ('software_render', 'sensor_safe')"
    ])

    # 无头模式默认使用 sensor_safe；需要硬件路径时可传 render_mode:=ogre。
    env = [
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', models),
        SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1', condition=IfCondition(software_render)),
        SetEnvironmentVariable('GALLIUM_DRIVER', 'llvmpipe', condition=IfCondition(software_render)),
        SetEnvironmentVariable('MESA_GL_VERSION_OVERRIDE', '4.5', condition=IfCondition(software_render)),
        SetEnvironmentVariable('MESA_GLSL_VERSION_OVERRIDE', '450', condition=IfCondition(software_render)),
    ]

    # 无头 gz sim（-s，避免 GUI 渲染卡住）
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gz_pkg, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-r -s -v2 ' + world}.items(),
    )

    # v8.4 半决赛世界不含 patrol_bot，需在此 spawn（与 bringup/nav_bringup 一致）。
    # 建图从场地左下角 (0.4, 0.4) 朝东起步，与 coverage_mapping.py 的 odom 航点假设一致。
    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(robot_pkg, 'launch', 'spawn_robot.launch.py')),
        launch_arguments={'spawn': 'true', 'x': '0.4', 'y': '0.4', 'yaw': '0.0'}.items(),
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/model/patrol_bot/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/model/patrol_bot/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            'lidar@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        remappings=[
            ('/model/patrol_bot/cmd_vel', '/cmd_vel'),
            ('/model/patrol_bot/odometry', '/odom'),
            ('lidar', '/scan'),
        ],
        output='screen',
    )

    frame_remap = Node(
        package='robot_description',
        executable='frame_remap.py',
        name='frame_remap',
        output='screen',
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(slam_pkg, 'launch', 'online_async_launch.py')),
        launch_arguments={'slam_params_file': slam_params, 'use_sim_time': 'true'}.items(),
    )

    # 默认用 explore_mapping.py（低速直线 + 原地转向，漂移明显小于随机避障的 auto_mapping.py）
    explorer = Node(
        package='robot_description',
        executable=LaunchConfiguration('explorer'),
        name='mapping_explorer',
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'render_mode',
            default_value='sensor_safe',
            description='sensor_safe/software_render 启用 llvmpipe；ogre 使用硬件渲染路径',
        ),
        DeclareLaunchArgument(
            'explorer',
            default_value='explore_mapping.py',
            description='左下角起步探索；START 起步覆盖请用 aic_mapping_demo.launch.py',
        ),
        *env, gz_sim, spawn_robot, bridge, frame_remap, slam, explorer,
    ])
