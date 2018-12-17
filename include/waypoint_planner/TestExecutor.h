#ifndef WAYPOINT_PLANNER_TEST_EXECUTOR_H_
#define WAYPOINT_PLANNER_TEST_EXECUTOR_H_

// ROS
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <waypoint_planner/ChangeTime.h>

#include "waypoint_planner/SMDPSolver.h"

class TestExecutor
{
public:
    TestExecutor(double horizon, double step, uint8_t mode, std::vector<double> waypoints = {});

    void run(double sim_step);

private:
  ros::NodeHandle n, pnh;

  ros::Publisher robot_vis_publisher;

  ros::ServiceClient human_sim_time_client;

  SMDPSolver solver;

  geometry_msgs::Point waypoint;
  double time_horizon;
  double time_step;
  double current_time;
  double next_decision;

  Action current_action;

  visualization_msgs::Marker robot_marker;
};

#endif  // WAYPOINT_PLANNER_TEST_EXECUTOR_H_
