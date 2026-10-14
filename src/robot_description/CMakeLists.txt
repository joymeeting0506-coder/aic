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
  scripts/coverage_mapping.py
  scripts/wasd_teleop.py
  scripts/reset_robot.py
  scripts/traffic_light_detector.py
  scripts/vision_detector.py
  scripts/waypoint_patrol.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

