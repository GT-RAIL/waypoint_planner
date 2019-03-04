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
#include <std_msgs/Float32.h>
#include <visualization_msgs/MarkerArray.h>

#include "waypoint_planner/EnvironmentSetup.h"

class HumanSimulator
{
public:
    HumanSimulator();

    void publishTFs();

    void advanceTime(double timestep);

private:

  void timeUpdateCallback(const std_msgs::Float32::ConstPtr& msg);

  void createTaskMarkers(int task);

  void addHumanMarker(std::string frame, double r, double g, double b);

  ros::NodeHandle n, pnh;

  ros::Publisher human_marker_publisher;
  ros::Publisher task_markers_publisher;
  ros::Subscriber time_update_subscriber;

  tf2_ros::TransformBroadcaster tf_broadcaster;

  HumanTrajectory trajectory;
  double time;
  double speed_factor;

  visualization_msgs::Marker human_marker;
  visualization_msgs::MarkerArray task_markers;

  geometry_msgs::TransformStamped task_tf1;
  geometry_msgs::TransformStamped task_tf2;
  geometry_msgs::TransformStamped task_tf3;
};

#endif  // WAYPOINT_PLANNER_HUMAN_SIMULATOR_H_
