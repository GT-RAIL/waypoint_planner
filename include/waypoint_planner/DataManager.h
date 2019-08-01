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
#include "waypoint_planner/SaveTensor1D.h"
#include "waypoint_planner/SaveTensor3D.h"

class DataManager
{
public:
    DataManager();

    void testReadData();

    void unpackLine(std::string line, Action &action, std::vector<double> &cost_constraints, PerchState &state,
        std::vector<geometry_msgs::Pose> &trajectory);

private:
  ros::NodeHandle pnh;
  ros::NodeHandle n;

  ros::ServiceClient save_1D_tensor;
  ros::ServiceClient save_3D_tensor;

  std::string file_path;
  std::string data_filename;
  bool classifier_mode;

  Approximator approximator;
};

#endif  // WAYPOINT_PLANNER_HUMAN_SIMULATOR_H_
