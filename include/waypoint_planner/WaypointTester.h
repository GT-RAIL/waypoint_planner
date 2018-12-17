#ifndef WAYPOINT_PLANNER_WAYPOINT_TESTER_H_
#define WAYPOINT_PLANNER_WAYPOINT_TESTER_H_

// CPP
#include <fstream>

// ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3.h>
#include <interactive_markers/interactive_marker_server.h>
#include <tf2_ros/transform_broadcaster.h>

#include "waypoint_planner/RewardsAndCosts.h"
#include "waypoint_planner/EnvironmentSetup.h"

class WaypointTester
{
public:
    WaypointTester();

    void publish_tfs();

private:

  ros::NodeHandle n, pnh;

  tf2_ros::TransformBroadcaster tf_broadcaster;

  geometry_msgs::PoseStamped human_pose;
  geometry_msgs::Vector3 human_dims;
  std::vector<geometry_msgs::Point> waypoints;
  int current_waypoint;

  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> im_server;

  void initialize_markers();

  void pose_updated(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

  void waypoint_clicked(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

  void calculate_costs();
};

#endif  // WAYPOINT_PLANNER_WAYPOINT_TESTER_H_
