cmake_minimum_required(VERSION 3.8)
project(smart_community_gazebo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  worlds
  models
  maps
  launch
  reference
  tools
  config
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  README.md
  TEAM_HANDOFF.md
  VERSION.txt
  person_manifest.csv
  layout_manifest.json
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  launch_world.sh
  verify_project.sh
  scripts/traffic_light_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
