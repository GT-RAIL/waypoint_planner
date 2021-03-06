#ifndef WAYPOINT_PLANNER_TEST_EXECUTOR_H_
#define WAYPOINT_PLANNER_TEST_EXECUTOR_H_

// ROS
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <visualization_msgs/Marker.h>

#include "waypoint_planner/LPSolver.h"
//#include "waypoint_planner/MCTSSolver.h"
#include "waypoint_planner/MCTSRewardSolver.h"
#include "waypoint_planner/MCTSScalarizedSolver.h"
#include "waypoint_planner/SMDPSolver.h"

class TestExecutor
{
public:
    static const uint8_t SMDP;
    static const uint8_t LP_SOLVE;
    static const uint8_t LP_LOAD;
    static const uint8_t MCTS_CONSTRAINED;
    static const uint8_t MCTS_SCALARIZED;

    TestExecutor(double horizon, double step, uint8_t approach, uint8_t mode,
        std::vector<double> weights, size_t search_depth, std::string trajectory_file);

    bool reset(double horizon, std::string trajectory_file, std::string lp_model="results");

    bool run(double sim_step);

    void reportResults();

    bool retryLP(int scaling_type);

    void freeLP();

    // execution metrics
    double r;
    double c1;
    double c2;
    double c3;

private:
  ros::NodeHandle n, pnh;

  ros::Publisher robot_vis_publisher;
  ros::Publisher human_sim_time_publisher;

  SMDPSolver solver;
  LPSolver lp_solver;
//  MCTSSolver mcts_solver;
  MCTSRewardSolver mcts_reward_solver;
  MCTSScalarizedSolver mcts_scalarized_solver;

  double c1_hat;
  double c2_hat;
  double c3_hat;

  PerchState state;
  double time_horizon;
  double time_step;
  double current_time;
  double next_decision;

  double search_depth_time;

  double move_dur;
  double start_move_time;
  double dx;
  double dy;
  double dz;
  double startx;
  double starty;
  double startz;

  uint8_t mode;
  std::vector<double> weights;
  size_t search_depth;

  Action current_action;
  HumanTrajectory trajectory;
  geometry_msgs::Vector3 default_human_dims;

  visualization_msgs::Marker robot_marker;

  uint8_t approach;
};

#endif  // WAYPOINT_PLANNER_TEST_EXECUTOR_H_
