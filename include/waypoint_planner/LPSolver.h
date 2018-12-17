#ifndef WAYPOINT_PLANNER_LP_SOLVER_H_
#define WAYPOINT_PLANNER_LP_SOLVER_H_

#include <geometry_msgs/Vector3.h>
#include <ros/package.h>

#include <lpsolve/lp_lib.h>

#include "waypoint_planner/Action.h"
#include "waypoint_planner/EnvironmentSetup.h"
#include "waypoint_planner/HumanTrajectory.h"
#include "waypoint_planner/RewardsAndCosts.h"
#include "waypoint_planner/State.h"

class LPSolver
{
public:
  LPSolver(double horizon, double step);

  LPSolver(double horizon, double step, std::string trajectory_file_name, std::string waypoint_file_name);

  void loadTrajectory(std::string file_name);

  void loadWaypoints(std::string file_name);

  void initializeActions();

  Action getAction(geometry_msgs::Point s, double t);

  Action getAction(geometry_msgs::Point s, size_t t);

private:
  std::vector<geometry_msgs::Point> waypoints;
  std::vector<Action> actions;

  HumanTrajectory trajectory;

  double time_horizon;
  double time_step;

//  std::vector< std::vector<double> > utility_map;  // Indexed as [waypoint #][time_step #]
//  std::vector< std::vector<Action> > action_map;  // Indexed as [waypoint #][time_step #]

  geometry_msgs::Vector3 default_human_dims;

  size_t waypointToIndex(geometry_msgs::Point w);
};

#endif  // WAYPOINT_PLANNER_LP_SOLVER_H_
