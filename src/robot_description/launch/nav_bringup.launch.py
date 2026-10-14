import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
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
    params = os.path.join(robot_pkg, 'config', 'nav2_params.yaml')

    render_mode = LaunchConfiguration('render_mode')
    software_render = PythonExpression([
        "'", render_mode, "' in ('software_render', 'sensor_safe')"
    ])

    env = [
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', models),
        SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1', condition=IfCondition(software_render)),
        SetEnvironmentVariable('GALLIUM_DRIVER', 'llvmpipe', condition=IfCondition(software_render)),
        SetEnvironmentVariable('MESA_GL_VERSION_OVERRIDE', '4.5', condition=IfCondition(software_render)),
        SetEnvironmentVariable('MESA_GLSL_VERSION_OVERRIDE', '450', condition=IfCondition(software_render)),
    ]

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gz_pkg, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-r -s -v2 ' + world}.items(),
    )

    # v8.4 半决赛世界不含 patrol_bot，需在此 spawn（spawn_robot.launch.py 内部 create + robot_state_publisher）
    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(robot_pkg, 'launch', 'spawn_robot.launch.py')),
        launch_arguments={'spawn': 'true'}.items(),
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
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        remappings=[
            ('/model/patrol_bot/cmd_vel', '/cmd_vel'),
            ('/model/patrol_bot/odometry', '/odom'),
            ('lidar', '/scan'),
            ('camera', '/camera/image_raw'),
            ('camera_info', '/camera/camera_info'),
        ],
        output='screen',
    )

    frame_remap = Node(
        package='robot_description',
        executable='frame_remap.py',
        name='frame_remap',
        output='screen',
    )

    # 红绿灯动态切换 + 颜色识别（与 bringup.launch.py 对齐，无头巡检也具备）
    traffic_light = Node(
        package='smart_community_gazebo',
        executable='traffic_light_controller.py',
        name='traffic_light_controller',
        output='screen',
    )

    traffic_light_detector = Node(
        package='robot_description',
        executable='traffic_light_detector.py',
        name='traffic_light_detector',
        output='screen',
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_pkg, 'launch', 'localization_launch.py')),
        launch_arguments={'map': map_file, 'params_file': params, 'use_sim_time': 'true'}.items(),
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_pkg, 'launch', 'navigation_launch.py')),
        launch_arguments={'params_file': params, 'use_sim_time': 'true'}.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'render_mode',
            default_value='sensor_safe',
            description='sensor_safe/software_render 启用 llvmpipe；ogre 使用硬件渲染路径',
        ),
        *env, gz_sim, spawn_robot, bridge, frame_remap,
        traffic_light, traffic_light_detector, localization, navigation,
    ])
