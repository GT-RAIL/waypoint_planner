#ifndef WAYPOINT_PLANNER_TRAJECTORY_SETTER_H_
#define WAYPOINT_PLANNER_TRAJECTORY_SETTER_H_

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

#include <yaml-cpp/yaml.h>

#include "waypoint_planner/EnvironmentSetup.h"

class TrajectorySetter
{
public:
    TrajectorySetter();

    void publishTFs();

private:

  ros::NodeHandle n, pnh;

  tf2_ros::TransformBroadcaster tf_broadcaster;

  geometry_msgs::PoseStamped human_pose;
  geometry_msgs::Vector3 human_dims;

  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> im_server;

  YAML::Emitter yaml;

  void initializeMarkers();

  void poseUpdated(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

  void markerClicked(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
};

#endif  // WAYPOINT_PLANNER_TRAJECTORY_SETTER_H_
