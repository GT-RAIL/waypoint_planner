#ifndef WAYPOINT_PLANNER_LP_SOLVER_H_
#define WAYPOINT_PLANNER_LP_SOLVER_H_

#include <iostream>

#include <geometry_msgs/Vector3.h>
#include <ros/package.h>

#include <lpsolve/lp_lib.h>

#include "waypoint_planner/Action.h"
#include "waypoint_planner/EnvironmentSetup.h"
#include "waypoint_planner/HumanTrajectory.h"
#include "waypoint_planner/PerchState.h"
#include "waypoint_planner/RewardsAndCosts.h"
#include "waypoint_planner/SMDPFunctions.h"
#include "waypoint_planner/State.h"
#include "waypoint_planner/StateWithTime.h"

class LPSolver
{
public:
  LPSolver(double horizon, double step, std::string trajectory_file_name="iss_trajectory.yaml",
      std::string waypoint_file_name="iss_waypoints.csv");

  void loadTrajectory(std::string file_name);

  void loadWaypoints(std::string file_name);

  void constructModel(std::vector<double> total_costs);

  void constructModel(std::vector<double> total_costs, PerchState s0);

  void setScaling(int scaling_type);

  bool solveModel(double timeout);

  void freeModel();

  void loadModel(std::string file_name);

  Action getAction(PerchState s, double t);

  Action getAction(PerchState s, size_t t);

  void reset(double horizon, std::string trajectory_file_name, std::string output_file_modifier="results");

  void resolve(double horizon, double step, size_t t0);

private:
  std::vector<geometry_msgs::Point> waypoints;
  std::vector<PerchState> perch_states;
  std::vector<StateWithTime> states;
  std::vector<Action> actions;
  std::vector<double> ys;
  std::vector<double> total_costs;

  std::string output_file_modifier;

  std::vector< std::vector< std::vector<size_t> > > index_map;  // index_map[state_id][time_step][action_id]=i for ys[i]
  size_t num_variables;

  HumanTrajectory trajectory;

  double time_horizon;
  double time_step;
  size_t t0;
  size_t t_end;

  lprec *lp;

//  std::vector< std::vector<double> > utility_map;  // Indexed as [waypoint #][time_step #]
//  std::vector< std::vector<Action> > action_map;  // Indexed as [waypoint #][time_step #]

  geometry_msgs::Vector3 default_human_dims;

  size_t getIndex(size_t perch_state_id, size_t t, size_t action_id);

  size_t getIndex(size_t state_id, size_t action_id);

  size_t perchStateToIndex(PerchState s);

  size_t waypointToIndex(geometry_msgs::Point w);

  double reward(size_t state_id, size_t action_id, uint8_t mode);

  void costConstraint(uint8_t mode, double threshold);

  bool isValidAction(size_t state_id, size_t action_id);
};

#endif  // WAYPOINT_PLANNER_LP_SOLVER_H_
