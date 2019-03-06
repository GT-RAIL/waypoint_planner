#ifndef WAYPOINT_PLANNER_SMDP_SOLVER_H_
#define WAYPOINT_PLANNER_SMDP_SOLVER_H_

#include <geometry_msgs/Vector3.h>
#include <ros/package.h>

#include "waypoint_planner/Action.h"
#include "waypoint_planner/EnvironmentSetup.h"
#include "waypoint_planner/HumanTrajectory.h"
#include "waypoint_planner/RewardsAndCosts.h"
#include "waypoint_planner/SMDPFunctions.h"
#include "waypoint_planner/State.h"

class SMDPSolver
{
public:
  SMDPSolver(double horizon, double step, uint8_t mode);

  SMDPSolver(double horizon, double step, uint8_t mode, std::string trajectory_file_name, std::string waypoint_file_name, std::vector<double> weights = {});

  void loadTrajectory(std::string file_name);

  void loadWaypoints(std::string file_name);

  void backwardsInduction();

  Action getAction(PerchState s, double t);

  Action getAction(PerchState s, size_t t);

  void reset(double horizon, std::string trajectory_file_name);

private:
  uint8_t mode;
  std::vector<double> linearization_weights;

  std::vector<geometry_msgs::Point> waypoints;
  std::vector<PerchState> perch_states;
  std::vector<Action> actions;

  HumanTrajectory trajectory;

  double time_horizon;
  double time_step;

  std::vector< std::vector<double> > utility_map;  // Indexed as [waypoint #][time_step #]
  std::vector< std::vector<Action> > action_map;  // Indexed as [waypoint #][time_step #]

  geometry_msgs::Vector3 default_human_dims;

  size_t waypointToIndex(geometry_msgs::Point w);

  size_t perchStateToIndex(PerchState s);
};

#endif  // WAYPOINT_PLANNER_SMDP_SOLVER_H_
