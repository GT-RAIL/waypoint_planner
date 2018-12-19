#ifndef WAYPOINT_PLANNER_LP_SOLVER_H_
#define WAYPOINT_PLANNER_LP_SOLVER_H_

#include <geometry_msgs/Vector3.h>
#include <ros/package.h>

#include <lpsolve/lp_lib.h>

#include "waypoint_planner/Action.h"
#include "waypoint_planner/EnvironmentSetup.h"
#include "waypoint_planner/HumanTrajectory.h"
#include "waypoint_planner/RewardsAndCosts.h"
#include "waypoint_planner/SMDPFunctions.h"
#include "waypoint_planner/State.h"

class StateWithTime
{
public:
    size_t waypoint_id;
    size_t time_index;

    StateWithTime(size_t waypoint_id, size_t time_index);

    static StateWithTime indexToState(size_t i, size_t max_time_steps);

    static size_t stateToIndex(StateWithTime state, size_t max_time_steps);

    static size_t stateToIndex(size_t waypoint_id, size_t time_index, size_t max_time_steps);
};

class LPSolver
{
public:
  LPSolver(double horizon, double step, std::string trajectory_file_name="iss_trajectory.yaml",
      std::string waypoint_file_name="iss_waypoints.csv");

  void loadTrajectory(std::string file_name);

  void loadWaypoints(std::string file_name);

  void constructModel(std::vector<double> total_costs);

  Action getAction(geometry_msgs::Point s, double t);

  Action getAction(geometry_msgs::Point s, size_t t);

private:
  std::vector<geometry_msgs::Point> waypoints;
  std::vector<StateWithTime> states;
  std::vector<Action> actions;

  HumanTrajectory trajectory;

  double time_horizon;
  double time_step;
  size_t t_end;

  lprec *lp;

//  std::vector< std::vector<double> > utility_map;  // Indexed as [waypoint #][time_step #]
//  std::vector< std::vector<Action> > action_map;  // Indexed as [waypoint #][time_step #]

  geometry_msgs::Vector3 default_human_dims;

  size_t waypointToIndex(geometry_msgs::Point w);

  double reward(size_t state_id, size_t action_id, uint8_t mode);

  void costConstraint(uint8_t mode, double threshold);
};

#endif  // WAYPOINT_PLANNER_LP_SOLVER_H_
