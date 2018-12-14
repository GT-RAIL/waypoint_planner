#ifndef WAYPOINT_PLANNER_HUMAN_SIMULATOR_H_
#define WAYPOINT_PLANNER_HUMAN_SIMULATOR_H_

// CPP
#include <fstream>

// ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3.h>
#include <tf2_ros/transform_broadcaster.h>
#include <waypoint_planner/ChangeTime.h>

#include "waypoint_planner/EnvironmentSetup.h"

class HumanSimulator
{
public:
    HumanSimulator();

    void publishTFs();

    void advanceTime(double timestep);

private:

  ros::NodeHandle n, pnh;

  ros::Publisher human_marker_publisher;

  ros::ServiceServer time_server;

  tf2_ros::TransformBroadcaster tf_broadcaster;

  HumanTrajectory trajectory;
  double time;
  double speed_factor;

  visualization_msgs::Marker human_marker;

  bool timeCallback(waypoint_planner::ChangeTime::Request &req, waypoint_planner::ChangeTime::Response &res);
};

#endif  // WAYPOINT_PLANNER_HUMAN_SIMULATOR_H_
