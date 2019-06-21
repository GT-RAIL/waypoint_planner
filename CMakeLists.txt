cmake_minimum_required(VERSION 2.8.3)
project(waypoint_planner)

add_compile_options(-std=c++11)

#find_package(Torch REQUIRED PATHS /home/dekent/torch/libtorch)
find_package(Torch REQUIRED PATHS /usr/local/lib/python2.7/dist-packages/torch)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  cmake_modules
  geometry_msgs
  interactive_markers
  pcl_ros
  roscpp
  roslib
  std_msgs
  tf
  tf2_bullet
  tf2_geometry_msgs
  visualization_msgs
)



find_library(lpsolve_LIBRARIES NAMES liblpsolve55.so lpsolve55 PATH_SUFFIXES lp_solve)

message(${lpsolve_LIBRARIES})

if(NOT lpsolve_LIBRARIES)
  message(FATAL_ERROR "Cannot find lpsolve library")
endif()

#find_package(Eigen REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################

add_service_files(
  FILES
  ChangeTime.srv
)

generate_messages()

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
  src/WaypointPlanner.cpp src/RewardsAndCosts.cpp src/Action.cpp
)
target_link_libraries(waypoint_planner
  ${catkin_LIBRARIES}
#  ${EIGEN_INCLUDE_DIRS}
)

add_executable(waypoint_tester
  src/WaypointTester.cpp src/RewardsAndCosts.cpp src/EnvironmentSetup.cpp src/HumanTrajectory.cpp src/Action.cpp
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
add_dependencies(
  human_simulator
  ${PROJECT_NAME}_generate_messages_cpp
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

add_executable(time_sampler
  src/TimeSampler.cpp
)
target_link_libraries(time_sampler
  ${catkin_LIBRARIES}
)

add_executable(test_node
  src/TestNode.cpp
  src/State.cpp
  src/Action.cpp
)
target_link_libraries(test_node
  ${catkin_LIBRARIES}
)

add_executable(test_executor
  src/TestExecutor.cpp
  src/LPSolver.cpp
  src/SMDPSolver.cpp
  src/State.cpp
  src/Action.cpp
  src/EnvironmentSetup.cpp
  src/HumanTrajectory.cpp
  src/RewardsAndCosts.cpp
  src/SMDPFunctions.cpp
  src/StateWithTime.cpp
#  src/MCTSSolver.cpp
  src/MCTSRewardSolver.cpp
  src/MCTSScalarizedSolver.cpp
  src/PerchState.cpp
  src/Approximator.cpp
)
add_dependencies(
  test_executor
  ${PROJECT_NAME}_generate_messages_cpp
)
target_link_libraries(test_executor
  ${catkin_LIBRARIES}
  ${lpsolve_LIBRARIES}
  ${TORCH_LIBRARIES}
  yaml-cpp
)

add_executable(data_manager
  src/DataManager.cpp
  src/Action.cpp
  src/Approximator.cpp
  src/EnvironmentSetup.cpp
  src/HumanTrajectory.cpp
  src/PerchState.cpp
)
target_link_libraries(data_manager
  ${catkin_LIBRARIES}
  ${TORCH_LIBRARIES}
  yaml-cpp
)
#add_dependencies(
#        data_manager
#        ${PROJECT_NAME}_generate_messages_cpp
#)

#add_executable(example-app src/Approximator.cpp)
#target_link_libraries(example-app ${TORCH_LIBRARIES} ${catkin_LIBRARIES})
#set_property(TARGET example-app PROPERTY CXX_STANDARD 11)

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