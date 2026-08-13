cmake_minimum_required(VERSION 3.28.0)
project(mrs_precise_landing)

set(LIBRARIES
  MrsPreciseLanding_PreciseLanding
)

set(DEPENDENCIES
  rclcpp
  rclcpp_components
  mrs_lib
  geometry_msgs
  mrs_msgs
  Eigen3
)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(USE_ROS_TIMER 0)

if(${USE_ROS_TIMER})
  MESSAGE(WARNING "[mrs_precise_landing]: Compiling with ROS Timers. This can cause high CPU load in runtime.")
endif()

foreach(DEP IN LISTS DEPENDENCIES)
  find_package(${DEP} REQUIRED)
endforeach()

# PreciseLanding

add_library(MrsPreciseLanding_PreciseLanding SHARED src/precise_landing.cpp)

target_link_libraries(MrsPreciseLanding_PreciseLanding
  PUBLIC
    rclcpp::rclcpp
    rclcpp_components::component
    mrs_lib::mrs_lib
    ${geometry_msgss_TARGETS}
    ${mrs_msgs_TARGETS}
    ${Eigen_LIBRARIES}
)

rclcpp_components_register_node(MrsPreciseLanding_PreciseLanding
  PLUGIN "mrs_precise_landing::PreciseLanding"
  EXECUTABLE MrsPreciseLanding_Node
)

## --------------------------------------------------------------
## |                           Install                          |
## --------------------------------------------------------------

install(TARGETS ${LIBRARIES} EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(
  export_${PROJECT_NAME} HAS_LIBRARY_TARGET
)

ament_export_dependencies(
  ${DEPENDENCIES}
)

ament_package()
