cmake_minimum_required(VERSION 2.8.3)
project(waypoint_planner)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  cmake_modules
  geometry_msgs
  interactive_markers
  roscpp
  roslib
  tf
  tf2_bullet
  tf2_geometry_msgs
  visualization_msgs
)

#find_package(Eigen REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################

###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(include
  ${catkin_INCLUDE_DIRS}
#  ${EIGEN_INCLUDE_DIRS}
)

add_executable(waypoint_planner
  src/WaypointPlanner.cpp src/RewardsAndCosts.cpp
)
target_link_libraries(waypoint_planner
  ${catkin_LIBRARIES}
#  ${EIGEN_INCLUDE_DIRS}
)

add_executable(waypoint_tester
  src/WaypointTester.cpp src/RewardsAndCosts.cpp src/EnvironmentSetup.cpp src/HumanTrajectory.cpp
)
target_link_libraries(waypoint_tester
  ${catkin_LIBRARIES}
  yaml-cpp
)

add_executable(human_simulator
  src/HumanSimulator.cpp src/EnvironmentSetup.cpp src/HumanTrajectory.cpp
)
target_link_libraries(human_simulator
  ${catkin_LIBRARIES}
  yaml-cpp
)

add_executable(waypoint_setter
  src/WaypointSetter.cpp
)
target_link_libraries(waypoint_setter
  ${catkin_LIBRARIES}
)

add_executable(trajectory_setter
  src/TrajectorySetter.cpp src/EnvironmentSetup.cpp src/HumanTrajectory.cpp
)
target_link_libraries(trajectory_setter
  ${catkin_LIBRARIES}
  yaml-cpp
)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS waypoint_planner waypoint_tester
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  add_rostest_gtest(cost_function_tests test/cost_function_tests.test test/CostFunctionTests.cpp src/RewardsAndCosts.cpp)

  target_link_libraries(cost_function_tests ${catkin_LIBRARIES})
endif()