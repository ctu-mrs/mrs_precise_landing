cmake_minimum_required(VERSION 3.28.0)
project(mrs_precise_landing_gazebo)

set(DEPENDENCIES
  ament_cmake
  ros_gz
)

foreach(DEP IN LISTS DEPENDENCIES)
  find_package(${DEP} REQUIRED)
endforeach()

## --------------------------------------------------------------
## |                           Install                          |
## --------------------------------------------------------------

install(DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models
)

install(DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)

install(DIRECTORY tmux/
  USE_SOURCE_PERMISSIONS
  DESTINATION share/${PROJECT_NAME}/tmux
)

ament_export_dependencies(
  ${DEPENDENCIES}
)

ament_package()
