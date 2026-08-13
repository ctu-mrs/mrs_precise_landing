cmake_minimum_required(VERSION 3.28.0)
project(mrs_landing_pad_estimation)

set(LIBRARIES
  MrsLandingPadEstimation_LandingPadEstimation
)

set(DEPENDENCIES
  rclcpp
  rclcpp_components
  tf2
  tf2_ros
  mrs_lib
  geometry_msgs
  mrs_msgs
  nav_msgs
  std_msgs
  apriltag_msgs
  Eigen3
)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(USE_ROS_TIMER 0)

if(${USE_ROS_TIMER})
  MESSAGE(WARNING "[mrs_landing_pad_estimation]: Compiling with ROS Timers. This can cause high CPU load in runtime.")
endif()

# set the compile options to show code warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

foreach(DEP IN LISTS DEPENDENCIES)
  find_package(${DEP} REQUIRED)
endforeach()

# LandingPadEstimation

add_library(MrsLandingPadEstimation_LandingPadEstimation SHARED src/landing_pad_estimation.cpp)

target_link_libraries(MrsLandingPadEstimation_LandingPadEstimation
  PUBLIC
    rclcpp::rclcpp
    rclcpp_components::component
    tf2::tf2
    tf2_ros::tf2_ros
    mrs_lib::mrs_lib
    ${geometry_msgss_TARGETS}
    ${mrs_msgs_TARGETS}
    ${nav_msgs_TARGETS}
    ${std_msgs_TARGETS}
    ${apriltag_msgs_TARGETS}
    ${Eigen_LIBRARIES}
)

rclcpp_components_register_node(MrsLandingPadEstimation_LandingPadEstimation
  PLUGIN "mrs_landing_pad_estimation::LandingPadEstimation"
  EXECUTABLE MrsLandingPadEstimation_Node
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
