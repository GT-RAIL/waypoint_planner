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
  static const uint8_t REWARD_ONLY;
  static const uint8_t LINEARIZED_COST;

  SMDPSolver(double horizon, double step, uint8_t mode);

  SMDPSolver(double horizon, double step, uint8_t mode, std::string trajectory_file_name, std::string waypoint_file_name, std::vector<double> weights = {});

  void loadTrajectory(std::string file_name);

  void loadWaypoints(std::string file_name);

  void initializeActions();

  void backwardsInduction();

  Action getAction(geometry_msgs::Point s, double t);

  Action getAction(geometry_msgs::Point s, size_t t);

private:
  uint8_t mode;
  std::vector<double> linearization_weights;

  std::vector<geometry_msgs::Point> waypoints;
  std::vector<Action> actions;

  HumanTrajectory trajectory;

  double time_horizon;
  double time_step;

  std::vector< std::vector<double> > utility_map;  // Indexed as [waypoint #][time_step #]
  std::vector< std::vector<Action> > action_map;  // Indexed as [waypoint #][time_step #]

  geometry_msgs::Vector3 default_human_dims;

  double reward(State s, Action a);

  void transitionModel(geometry_msgs::Point s, Action a, std::vector<geometry_msgs::Point> &s_primes,
      std::vector<double> &probabilities);

  size_t waypointToIndex(geometry_msgs::Point w);

  double linearizedCost(geometry_msgs::Pose h, geometry_msgs::Vector3 human_dims, geometry_msgs::Point r);
};

#endif  // WAYPOINT_PLANNER_SMDP_SOLVER_H_
