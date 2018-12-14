#ifndef WAYPOINT_PLANNER_SMDP_SOLVER_H_
#define WAYPOINT_PLANNER_SMDP_SOLVER_H_

#include <geometry_msgs/Vector3.h>
#include <ros/package.h>

#include "waypoint_planner/Action.h"
#include "waypoint_planner/EnvironmentSetup.h"
#include "waypoint_planner/HumanTrajectory.h"
#include "waypoint_planner/RewardsAndCosts.h"
#include "waypoint_planner/State.h"

class SMDPSolver
{
public:
  SMDPSolver(double horizon, double step);

  SMDPSolver(double horizon, double step, std::string trajectory_file_name, std::string waypoint_file_name);

  void loadTrajectory(std::string file_name);

  void loadWaypoints(std::string file_name);

  void initializeActions();

  void backwardsInduction();

  Action get_action(geometry_msgs::PointStamped s, double t);

  Action get_action(geometry_msgs::PointStamped s, size_t t);

private:
  std::vector<geometry_msgs::PointStamped> waypoints;
  std::vector<Action> actions;

  HumanTrajectory trajectory;

  double time_horizon;
  double time_step;

  std::vector< std::vector<double> > utility_map;  // Indexed as [waypoint #][time_step #]
  std::vector< std::vector<Action> > action_map;  // Indexed as [waypoint #][time_step #]

  geometry_msgs::Vector3 default_human_dims;

  double reward(State s, Action a);

  void transition_model(geometry_msgs::PointStamped s, Action a, std::vector<geometry_msgs::PointStamped> &s_primes,
      std::vector<double> &probabilities);

  size_t waypoint_to_index(geometry_msgs::PointStamped w);
};

#endif  // WAYPOINT_PLANNER_SMDP_SOLVER_H_
