#ifndef WAYPOINT_PLANNER_MCTS_SCALARIZED_SOLVER_H_
#define WAYPOINT_PLANNER_MCTS_SCALARIZED_SOLVER_H_

#include <chrono>
#include <iostream>
#include <mutex>
#include <random>
#include <thread>

#include <boost/functional/hash.hpp>
#include <boost/range/adaptor/map.hpp>

#include <geometry_msgs/Vector3.h>
#include <ros/package.h>
#include <ros/time.h>

#include "waypoint_planner/Action.h"
#include "waypoint_planner/EnvironmentSetup.h"
#include "waypoint_planner/HumanTrajectory.h"
#include "waypoint_planner/PerchState.h"
#include "waypoint_planner/RewardsAndCosts.h"
#include "waypoint_planner/SMDPFunctions.h"
#include "waypoint_planner/State.h"
#include "waypoint_planner/StateWithTime.h"

class MCTSScalarizedSolver
{
public:
    MCTSScalarizedSolver(double horizon, double step, std::string trajectory_file_name,
      std::string waypoint_file_name, std::vector<double> weights, double timeout_sec=10.0,
      size_t max_time_step_search_depth=150, double exploration_constant=1.0, int num_threads = 1);

  Action search(PerchState s, double t);

  Action search(size_t state_index, size_t time_step);

  Action search(StateWithTime s0);

  void loadTrajectory(std::string file_name);

  void loadWaypoints(std::string file_name);

  Action getAction(PerchState s, double t);

  Action getAction(PerchState s, size_t t);

  void setWeights(std::vector<double> weights);

  void reset(double horizon, std::string trajectory_file_name);

private:
  static const uint64_t XORSHIFT_MAX;

  std::vector<geometry_msgs::Point> waypoints;
  std::vector<PerchState> perch_states;
  std::vector<StateWithTime> states;
  std::vector<Action> actions;

  boost::hash< std::vector<double> > hasher;
  std::map<double, size_t> state_index_map;

  std::vector< std::vector< std::vector<size_t> > > index_map;  // index_map[state_id][time_step][action_id]=i for ys[i]
  size_t num_variables;

  HumanTrajectory trajectory;

  double time_horizon;
  double time_step;
  size_t t_end;

  geometry_msgs::Vector3 default_human_dims;

  size_t num_costs;

  // Scalarization function weights
  std::vector<double> weights;

  // UCT parameters
  double exploration_constant;
  ros::Duration timeout;
  size_t max_search_time_step;
  size_t max_search_time_step_dst;

  // lookup tables for rollouts
  std::map<size_t, double> N_s;
  std::map<size_t, double> N_sa;
  std::map<size_t, double> QR_sa;

  // random number generators
//  std::mt19937 generator;
//  std::uniform_real_distribution<double> uniform_dist;
//  std::bernoulli_distribution bernoulli_dist;
  uint64_t xorshift_state;

  // multithreading
  std::mutex rand_mutex;
  std::mutex lookup_mutex;
  int num_threads;

  std::vector<double> simulate(StateWithTime s);

  size_t greedyPolicy(StateWithTime s);

  size_t selectAction(StateWithTime s, double kappa);

  StateWithTime simulate_action(StateWithTime s, Action a, std::vector<double> &result_costs);

  size_t uniformSelect(std::vector<size_t> &actions);

  size_t getIndexSA(size_t perch_state_id, size_t t, size_t action_id);

  size_t getIndexSA(size_t state_id, size_t action_id);  // state_id: index in states vector (s(t))

  size_t getIndexS(size_t perch_state_id, size_t t);

  size_t getIndexS(StateWithTime s);

//  double reward(size_t state_id, size_t action_id, uint8_t mode);

  bool isValidAction(size_t waypoint_id, size_t action_id);

  size_t perchStateToIndex(PerchState s);

  size_t stateHash(PerchState s);

  uint64_t xorshift64();
};

#endif  // WAYPOINT_PLANNER_MCTS_SCALARIZED_SOLVER_H_
