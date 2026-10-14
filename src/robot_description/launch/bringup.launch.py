import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    world_pkg = get_package_share_directory('smart_community_gazebo')
    robot_pkg = get_package_share_directory('robot_description')
    gz_pkg = get_package_share_directory('ros_gz_sim')
    nav2_pkg = get_package_share_directory('nav2_bringup')

    world = os.path.join(world_pkg, 'worlds', 'smart_community_v2.sdf')
    models = os.path.join(world_pkg, 'models')
    map_file = os.path.join(world_pkg, 'maps', 'community.yaml')
    nav2_params = os.path.join(robot_pkg, 'config', 'nav2_params.yaml')

    start_navigation = LaunchConfiguration('start_navigation')
    start_traffic_detector = LaunchConfiguration('start_traffic_detector')
    robot_scale = LaunchConfiguration('robot_scale')
    headless = LaunchConfiguration('headless')
    render_mode = LaunchConfiguration('render_mode')
    software_render = PythonExpression([
        "'", render_mode, "' in ('software_render', 'sensor_safe')"
    ])
    gui_software = PythonExpression([
        "'", headless, "' != 'true' and ", software_render
    ])
    gui_hardware = PythonExpression([
        "'", headless, "' != 'true' and not ", software_render
    ])

    # GUI 默认使用 VMware 下已验证稳定的 ogre。software_render/sensor_safe
    # 则启用 llvmpipe，并使用 ogre2 解决 gpu_lidar/camera 的 FBO 问题。
    env = [
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', models),
        SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1', condition=IfCondition(software_render)),
        SetEnvironmentVariable('GALLIUM_DRIVER', 'llvmpipe', condition=IfCondition(software_render)),
        SetEnvironmentVariable('MESA_GL_VERSION_OVERRIDE', '4.5', condition=IfCondition(software_render)),
        SetEnvironmentVariable('MESA_GLSL_VERSION_OVERRIDE', '450', condition=IfCondition(software_render)),
    ]

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gz_pkg, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-r -v2 --render-engine ogre ' + world}.items(),
        condition=IfCondition(gui_hardware),
    )

    gz_sim_software = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gz_pkg, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-r -v2 --render-engine ogre2 ' + world}.items(),
        condition=IfCondition(gui_software),
    )

    # headless:=true 时用 -s（只跑服务端，不开 GUI），传感器照常出数据。
    # 本机软渲染下 GUI 会吃掉大量 CPU，调试/跑算法时用无头模式更快。
    gz_sim_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gz_pkg, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-r -s -v2 ' + world}.items(),
        condition=IfCondition(headless),
    )

    # v8.4 半决赛世界（smart_community_semifinal_v8）不含 patrol_bot，需在此 spawn。
    # spawn_robot.launch.py 内部用 ros_gz_sim create 按 x/y/yaw 生成机器人，
    # 同时启动 robot_state_publisher。
    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(robot_pkg, 'launch', 'spawn_robot.launch.py')),
        launch_arguments={'spawn': 'true', 'scale': robot_scale}.items(),
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/model/patrol_bot/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/model/patrol_bot/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            'lidar@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            'camera@sensor_msgs/msg/Image[gz.msgs.Image',
            'camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            'imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/world/smart_community_semifinal_v8/model/patrol_bot/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        # 不桥接 gz 的 /model/patrol_bot/tf：TF 树由 robot_state_publisher（URDF 静态链）
        # 和 frame_remap（odom->base_link）负责，gz 那套 patrol_bot::... 的位姿 TF 无人消费。
        remappings=[
            ('/model/patrol_bot/cmd_vel', '/cmd_vel'),
            ('/model/patrol_bot/odometry', '/odom'),
            ('lidar', '/scan'),
            ('camera', '/camera/image_raw'),
            ('camera_info', '/camera/camera_info'),
            ('imu', '/imu/data'),
            ('/world/smart_community_semifinal_v8/model/patrol_bot/joint_state', '/joint_states'),
        ],
        output='screen',
    )

    traffic_light = Node(
        package='smart_community_gazebo',
        executable='traffic_light_controller.py',
        name='traffic_light_controller',
        output='screen',
    )

    frame_remap = Node(
        package='robot_description',
        executable='frame_remap.py',
        name='frame_remap',
        output='screen',
    )

    traffic_light_detector = Node(
        package='robot_description',
        executable='traffic_light_detector.py',
        name='traffic_light_detector',
        output='screen',
        condition=IfCondition(start_traffic_detector),
    )

    # Nav2 定位 + 导航（与 nav_bringup.launch.py 一致），让 GUI 快捷命令也能直接跑巡检。
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_pkg, 'launch', 'localization_launch.py')),
        condition=IfCondition(start_navigation),
        launch_arguments={'map': map_file, 'params_file': nav2_params, 'use_sim_time': 'true'}.items(),
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_pkg, 'launch', 'navigation_launch.py')),
        condition=IfCondition(start_navigation),
        launch_arguments={'params_file': nav2_params, 'use_sim_time': 'true'}.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('start_navigation', default_value='true'),
        DeclareLaunchArgument('start_traffic_detector', default_value='true'),
        DeclareLaunchArgument('robot_scale', default_value='0.5'),
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='true 则不开 Gazebo GUI（-s 无头模式），传感器数据照常输出',
        ),
        DeclareLaunchArgument(
            'render_mode',
            default_value='ogre',
            description='ogre（稳定 GUI）、software_render 或 sensor_safe（llvmpipe+ogre2）',
        ),
        *env, gz_sim, gz_sim_software, gz_sim_headless,
        spawn_robot, bridge, traffic_light, frame_remap, traffic_light_detector,
        localization, navigation,
    ])
