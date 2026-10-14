"""AIC Jazzy Nav2 navigation servers, no docking/route server.

Based on ROS Navigation2 jazzy/nav2_bringup/launch/navigation_launch.py;
this project does not currently use docking or Nav2 graph route service.
Conventional Nav2 through-poses functionality remains enabled.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key='',
            param_rewrites={'autostart': autostart},
            convert_types=True,
        ),
        allow_substs=True,
    )
    remaps = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    def nav_node(package, executable, name, extra_remaps=()):
        return Node(
            package=package, executable=executable,
            name=name, output='screen', parameters=[configured_params],
            remappings=remaps + list(extra_remaps),
        )

    node_names = [
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'velocity_smoother',
        'collision_monitor',
        'bt_navigator',
        'waypoint_follower',
    ]
    actions = [
        SetParameter('use_sim_time', use_sim_time),
        nav_node('nav2_controller', 'controller_server', 'controller_server', [('cmd_vel', 'cmd_vel_nav')]),
        nav_node('nav2_smoother', 'smoother_server', 'smoother_server'),
        nav_node('nav2_planner', 'planner_server', 'planner_server'),
        nav_node('nav2_behaviors', 'behavior_server', 'behavior_server', [('cmd_vel', 'cmd_vel_nav')]),
        nav_node('nav2_velocity_smoother', 'velocity_smoother', 'velocity_smoother', [('cmd_vel', 'cmd_vel_nav')]),
        nav_node('nav2_collision_monitor', 'collision_monitor', 'collision_monitor'),
        nav_node('nav2_bt_navigator', 'bt_navigator', 'bt_navigator'),
        nav_node('nav2_waypoint_follower', 'waypoint_follower', 'waypoint_follower'),
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_navigation', output='screen',
            parameters=[{'autostart': autostart}, {'node_names': node_names}],
        ),
    ]
    return LaunchDescription([
        DeclareLaunchArgument('params_file', description='v3 Nav2 parameters YAML'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        GroupAction(actions),
    ])
