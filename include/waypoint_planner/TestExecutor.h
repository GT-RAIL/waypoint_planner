#ifndef WAYPOINT_PLANNER_TEST_EXECUTOR_H_
#define WAYPOINT_PLANNER_TEST_EXECUTOR_H_

// ROS
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <visualization_msgs/Marker.h>

#include "waypoint_planner/LPSolver.h"
#include "waypoint_planner/MCTSSolver.h"
#include "waypoint_planner/SMDPSolver.h"

class TestExecutor
{
public:
    static const uint8_t SMDP;
    static const uint8_t LP_SOLVE;
    static const uint8_t LP_LOAD;
    static const uint8_t MCTS;

    TestExecutor(double horizon, double step, uint8_t approach, uint8_t mode=SMDPFunctions::LINEARIZED_COST,
        std::vector<double> weights = {});

    bool run(double sim_step);

    void reportResults();

private:
  ros::NodeHandle n, pnh;

  ros::Publisher robot_vis_publisher;
  ros::Publisher human_sim_time_publisher;

  SMDPSolver solver;
  LPSolver lp_solver;
  MCTSSolver mcts_solver;

  geometry_msgs::Point waypoint;
  double time_horizon;
  double time_step;
  double current_time;
  double next_decision;

  // execution metrics
  double r;
  double c1;
  double c2;

  Action current_action;
  HumanTrajectory trajectory;
  geometry_msgs::Vector3 default_human_dims;

  visualization_msgs::Marker robot_marker;

  uint8_t approach;
};

#endif  // WAYPOINT_PLANNER_TEST_EXECUTOR_H_
