#ifndef WAYPOINT_PLANNER_DATA_MANAGER_H_
#define WAYPOINT_PLANNER_DATA_MANAGER_H_

// CPP
#include <fstream>

// ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/PoseStamped.h>

#include "waypoint_planner/Action.h"
#include "waypoint_planner/Approximator.h"
#include "waypoint_planner/PerchState.h"

class DataManager
{
public:
    DataManager();

    void readFile();

    void unpackLine(std::string line);

private:
  ros::NodeHandle pnh;

  std::string file_path;
};

#endif  // WAYPOINT_PLANNER_HUMAN_SIMULATOR_H_
