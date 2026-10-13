cmake_minimum_required(VERSION 3.8)
project(robot_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    urdf
    launch
    config
  DESTINATION
    share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/frame_remap.py
  scripts/explore_mapping.py
  scripts/auto_mapping.py
  scripts/wasd_teleop.py
  scripts/reset_robot.py
  scripts/route_mapping.py
  scripts/make_nav2_v5_params.py
  scripts/aic_v5_vision.py
  scripts/aic_v5_patrol.py
  scripts/aic_v5_audit.py
  scripts/aic_v5_route.py
  scripts/map_segment_gate.py
  scripts/route_preflight.py
  scripts/lane_geometry_preflight.py
  scripts/nav_patrol.py
  scripts/initial_pose.py
  scripts/traffic_light_detector.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

