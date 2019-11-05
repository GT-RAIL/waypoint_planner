#ifndef WAYPOINT_PLANNER_RANDOM_BASELINE_H_
#define WAYPOINT_PLANNER_RANDOM_BASELINE_H_

#include <chrono>
#include <random>

#include <geometry_msgs/Vector3.h>
#include <ros/package.h>

#include "waypoint_planner/Action.h"
#include "waypoint_planner/EnvironmentSetup.h"
#include "waypoint_planner/HumanTrajectory.h"
#include "waypoint_planner/RewardsAndCosts.h"
#include "waypoint_planner/SMDPFunctions.h"
#include "waypoint_planner/State.h"

class RandomBaseline
{
public:
  RandomBaseline(std::string waypoint_file_name);

  void loadWaypoints(std::string file_name);

  Action getAction(PerchState s);

private:
  std::vector<geometry_msgs::Point> waypoints;
  std::vector<Action> actions;

  std::mt19937 generator;
  std::uniform_real_distribution<double> uniform_dist;
};

#endif  // WAYPOINT_PLANNER_RANDOM_BASELINE_H_
