#ifndef WAYPOINT_PLANNER_WAYPOINT_SETTER_H_
#define WAYPOINT_PLANNER_WAYPOINT_SETTER_H_

// ROS
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3.h>
#include <interactive_markers/interactive_marker_server.h>
#include <tf2_ros/transform_broadcaster.h>

#include "waypoint_planner/RewardsAndCosts.h"

class WaypointSetter
{
public:
    WaypointSetter();

private:

  ros::NodeHandle n, pnh;

  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> im_server;

  void pose_updated(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
};

#endif  // WAYPOINT_PLANNER_WAYPOINT_SETTER_H_
