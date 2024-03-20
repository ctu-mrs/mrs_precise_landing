cmake_minimum_required(VERSION 3.5)
project(mrs_landing_pad_estimation)

set(LIBRARIES
  MrsLandingPadEstimation_LandingPadEstimation
  )

set(CATKIN_DEPENDENCIES
  cmake_modules
  geometry_msgs
  mrs_lib
  mrs_msgs
  nav_msgs
  nodelet
  roscpp
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  apriltag_ros
  )

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions(-Wall)
add_definitions(-Wextra)

find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${Eigen_LIBRARIES})

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_DEPENDENCIES}
  )

catkin_package(
  CATKIN_DEPENDS ${CATKIN_DEPENDENCIES}
  LIBRARIES ${LIBRARIES}
  DEPENDS Eigen
  )

include_directories(
  ${catkin_INCLUDE_DIRS}
  )

# BrickEstimation

add_library(MrsLandingPadEstimation_LandingPadEstimation src/landing_pad_estimation.cpp)

add_dependencies(MrsLandingPadEstimation_LandingPadEstimation
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(MrsLandingPadEstimation_LandingPadEstimation
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  )
