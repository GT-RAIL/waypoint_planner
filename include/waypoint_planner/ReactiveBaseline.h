#ifndef WAYPOINT_PLANNER_REACTIVE_BASELINE_H_
#define WAYPOINT_PLANNER_REACTIVE_BASELINE_H_

#include <geometry_msgs/Vector3.h>
#include <ros/package.h>

#include "waypoint_planner/Action.h"
#include "waypoint_planner/EnvironmentSetup.h"
#include "waypoint_planner/HumanTrajectory.h"
#include "waypoint_planner/RewardsAndCosts.h"
#include "waypoint_planner/SMDPFunctions.h"
#include "waypoint_planner/State.h"

class ReactiveBaseline
{
public:
  ReactiveBaseline(std::string waypoint_file_name, std::vector<double> weights={});

  void loadTrajectory(std::string file_name);

  void setWeights(std::vector<double> weights);

  void setTrajectory(std::vector<HumanTrajectory> trajectories);

  void loadWaypoints(std::string file_name);

  Action getAction(PerchState s, double t);

private:
  std::vector<double> linearization_weights;

  std::vector<geometry_msgs::Point> waypoints;
  std::vector<PerchState> perch_states;
  std::vector<Action> actions;

  std::vector<HumanTrajectory> trajectories;

  geometry_msgs::Vector3 default_human_dims;
};

#endif  // WAYPOINT_PLANNER_REACTIVE_BASELINE_H_
