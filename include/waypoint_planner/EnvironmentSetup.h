#ifndef WAYPOINT_PLANNER_ENVIRONMENT_SETUP_H_
#define WAYPOINT_PLANNER_ENVIRONMENT_SETUP_H_

#include <chrono>
#include <fstream>
#include <random>
#include <stdlib.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <visualization_msgs/Marker.h>

#include <yaml-cpp/yaml.h>

#include "waypoint_planner/HumanTrajectory.h"

class EnvironmentSetup
{
public:
  static void readWaypoints(std::string file_path, std::vector<geometry_msgs::Point> &waypoints);

  static HumanTrajectory readHumanTrajectory(std::string file_path, bool interpolate=true, double step=0.033333,
      bool randomize_trajectory=false);

  static void sampleHumanTrajectories(const std::string &file_path, std::vector<HumanTrajectory> &sampled_trajectories,
      int num_samples=1, long seed = 0, bool interpolate=true, double step=0.033333);

  static visualization_msgs::Marker initializeHumanMarker();

  static visualization_msgs::Marker initializeHumanMarker(geometry_msgs::Vector3 human_dims);
};

#endif  // WAYPOINT_PLANNER_ENVIRONMENT_SETUP_H_