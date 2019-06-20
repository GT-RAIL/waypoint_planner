#ifndef WAYPOINT_PLANNER_APPROXIMATOR_H_
#define WAYPOINT_PLANNER_APPROXIMATOR_H_

#include <ros/ros.h>
#include <fstream>
#include <stdlib.h>

#include <geometry_msgs/Pose.h>
#include <pcl_ros/point_cloud.h>

#include <torch/torch.h>

#include <pcl/common/common.h>
#include <pcl/point_cloud.h>

#include "waypoint_planner/Action.h"
#include "waypoint_planner/PerchState.h"

class Approximator
{
public:
  Approximator(int window=90, double min_x=9, double max_x=12.5, double min_y=-12, double max_y=-2, double min_z=3,
      double max_z=7);
//  Approximator(int window=60, double min_x=0, double max_x=20, double min_y=-20, double max_y=10, double min_z=-5,
//      double max_z=15);

  void createInput(std::vector<double> cost_constraints, PerchState robot_state,
      std::vector<geometry_msgs::Pose> trajectory);

private:
  static const size_t IMAGE_SIZE;
  double min_x, min_y, min_z, max_x, max_y, max_z;
  double x_range, y_range, z_range;
  int window;

  ros::NodeHandle pn;
  ros::Publisher img_cloud_pub;

  int xIndex(double x);

  int yIndex(double y);

  int zIndex(double z);

  int rotIndex(double rot);

  std::vector< std::vector<int> > interpolatePoints(int x1, int x2, int y1, int y2, int z1, int z2);
};

#endif  // WAYPOINT_PLANNER_APPROXIMATOR_H_