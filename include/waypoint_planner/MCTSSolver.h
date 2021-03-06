#ifndef WAYPOINT_PLANNER_MCTS_SOLVER_H_
#define WAYPOINT_PLANNER_MCTS_SOLVER_H_

#include <iostream>

#include <boost/functional/hash.hpp>

#include <geometry_msgs/Vector3.h>
#include <ros/package.h>
#include <ros/time.h>

#include "waypoint_planner/Action.h"
#include "waypoint_planner/EnvironmentSetup.h"
#include "waypoint_planner/HumanTrajectory.h"
#include "waypoint_planner/RewardsAndCosts.h"
#include "waypoint_planner/SMDPFunctions.h"
#include "waypoint_planner/State.h"
#include "waypoint_planner/StateWithTime.h"

class MCTSSolver
{
public:
  MCTSSolver(double horizon, double step, std::string trajectory_file_name,
      std::string waypoint_file_name, std::vector<double> constraints, double timeout_sec=10.0, size_t max_time_step_search_depth=150, double exploration_constant=1.0);

  Action search(geometry_msgs::Point w, double t);

  Action search(size_t waypoint_index, size_t time_step);

  Action search(StateWithTime s0);

  void loadTrajectory(std::string file_name);

  void loadWaypoints(std::string file_name);

  Action getAction(geometry_msgs::Point s, double t);

  Action getAction(geometry_msgs::Point s, size_t t);

  void updateConstraints(geometry_msgs::Point s, double t);

  void setConstraints(std::vector<double> constraints);

private:
  std::vector<geometry_msgs::Point> waypoints;
  std::vector<StateWithTime> states;
  std::vector<Action> actions;

  boost::hash< std::vector<double> > hasher;
  std::map<double, size_t> waypoint_index_map;

  std::vector< std::vector< std::vector<size_t> > > index_map;  // index_map[state_id][time_step][action_id]=i for ys[i]
  size_t num_variables;

  HumanTrajectory trajectory;

  double time_horizon;
  double time_step;
  size_t t_end;

  geometry_msgs::Vector3 default_human_dims;

  size_t num_costs;

  // Max expected accumulated costs
  std::vector<double> cost_constraints;

  // CCUCT gradient descent parameters
  std::vector<double> lambda;
  double learning_rate;  // a function of time
  double exploration_constant;
  ros::Duration timeout;
  size_t max_search_time_step;
  size_t max_search_time_step_dst;

  // lookup tables for rollouts
  std::map<size_t, double> N_s;
  std::map<size_t, double> N_sa;
  std::vector< std::map<size_t, double> > VC_s;
  std::map<size_t, double> QR_sa;
  std::vector< std::map<size_t, double> > QC_sa;

  std::vector<double> simulate(StateWithTime s);

  size_t greedyPolicy(StateWithTime s, double kappa);

  std::vector<double> rollout(StateWithTime s);

  StateWithTime simulate_action(StateWithTime s, Action a, std::vector<double> &result_costs);

  size_t getIndexSA(size_t waypoint_id, size_t t, size_t action_id);

  size_t getIndexSA(size_t state_id, size_t action_id);  // state_id: index in states vector (s(t))

  size_t getIndexS(size_t waypoint_id, size_t t);

  size_t getIndexS(StateWithTime s);

  double reward(size_t state_id, size_t action_id, uint8_t mode);

  bool isValidAction(size_t waypoint_id, size_t action_id);

  size_t waypointToIndex(geometry_msgs::Point w);

  size_t waypointHash(geometry_msgs::Point w);
};

#endif  // WAYPOINT_PLANNER_MCTS_SOLVER_H_
