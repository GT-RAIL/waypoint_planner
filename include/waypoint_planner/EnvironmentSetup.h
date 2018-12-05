#ifndef WAYPOINT_PLANNER_ENVIRONMENT_SETUP_H_
#define WAYPOINT_PLANNER_ENVIRONMENT_SETUP_H_

#include <fstream>
#include <stdlib.h>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3.h>
#include <visualization_msgs/Marker.h>

#include <yaml-cpp/yaml.h>

#include "waypoint_planner/HumanTrajectory.h"

class EnvironmentSetup
{
public:
  static void readWaypoints(std::string file_path, std::vector<geometry_msgs::PointStamped> &waypoints);

  static HumanTrajectory readHumanTrajectory(std::string file_path, bool interpolate=true, double step=0.033333);

  static visualization_msgs::Marker initializeHumanMarker();

  static visualization_msgs::Marker initializeHumanMarker(geometry_msgs::Vector3 human_dims);
};

#endif  // WAYPOINT_PLANNER_ENVIRONMENT_SETUP_H_