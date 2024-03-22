cmake_minimum_required(VERSION 3.1.2)
project(mrs_precise_landing)

set(LIBRARIES
  MrsPreciseLanding_PreciseLanding
  )

set(CATKIN_DEPENDENCIES
  cmake_modules
  roscpp
  geometry_msgs
  mrs_lib
  nodelet
  mrs_msgs
  )

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${Eigen_LIBRARIES})

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_DEPENDENCIES}
  )

find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${EIGEN3_LIBRARIES})

catkin_package(
  CATKIN_DEPENDS ${CATKIN_DEPENDENCIES}
  DEPENDS Eigen
  LIBRARIES ${LIBRARIES}
  )

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  )

# PreciseLanding

add_library(MrsPreciseLanding_PreciseLanding src/precise_landing.cpp)

add_dependencies(MrsPreciseLanding_PreciseLanding
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(MrsPreciseLanding_PreciseLanding
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  )

## --------------------------------------------------------------
## |                           Install                          |
## --------------------------------------------------------------

install(TARGETS ${LIBRARIES}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

install(FILES nodelets.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
