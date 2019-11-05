#ifndef WAYPOINT_PLANNER_TEST_EXECUTOR_H_
#define WAYPOINT_PLANNER_TEST_EXECUTOR_H_

// ROS
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <visualization_msgs/Marker.h>

#include "waypoint_planner/Approximator.h"
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
    static const uint8_t RANDOM;
    static const uint8_t REACTIVE;
    static const uint8_t APPROXIMATE_STOCHASTIC;
    static const uint8_t APPROXIMATE_DETERMINISTIC;

    TestExecutor(double horizon, double step, uint8_t approach, uint8_t mode,
        std::vector<double> weights, size_t search_depth, std::string trajectory_file,
        size_t trajectory_samples = 100, bool lp_resolve = false, double resolve_horizon = 30);

    void randomizeWeights();

    bool reset(double horizon, std::string trajectory_file, std::string lp_model="results", bool solve=true,
        bool randomize_trajectory=false);

    // resolve LP from current time step and state
    bool resolve();

    bool run(double sim_step, std::string log_name="log.txt");

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

  bool lp_resolve;

  double c1_hat;
  double c2_hat;
  double c3_hat;

  PerchState state;
  double time_horizon;
  double time_step;
  double current_time;
  double next_decision;
  double resolve_horizon;

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
  std::vector<HumanTrajectory> trajectories;
  HumanTrajectory eval_trajectory;
  geometry_msgs::Vector3 default_human_dims;

  visualization_msgs::Marker robot_marker;

  uint8_t approach;
};

#endif  // WAYPOINT_PLANNER_TEST_EXECUTOR_H_
