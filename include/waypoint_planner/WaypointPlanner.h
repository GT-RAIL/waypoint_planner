#ifndef WAYPOINT_PLANNER_WAYPOINT_PLANNER_H_
#define WAYPOINT_PLANNER_WAYPOINT_PLANNER_H_

// ROS
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_listener.h>

#include "waypoint_planner/RewardsAndCosts.h"

class WaypointPlanner
{
public:
    WaypointPlanner();

private:

  ros::NodeHandle n, pnh;

  tf::TransformListener tfListener;
};

#endif  // WAYPOINT_PLANNER_WAYPOINT_PLANNER_H_
