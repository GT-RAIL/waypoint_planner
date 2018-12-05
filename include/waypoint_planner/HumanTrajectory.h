#ifndef WAYPOINT_PLANNER_HUMAN_TRAJECTORY_H_
#define WAYPOINT_PLANNER_HUMAN_TRAJECTORY_H_

// ROS
#include <geometry_msgs/PoseStamped.h>
#include <tf2_bullet/tf2_bullet.h>

class HumanTrajectory
{
public:
    HumanTrajectory();

    void addPose(double time, geometry_msgs::PoseStamped pose);

    void interpolate(double step);

    void sortKeys();

    geometry_msgs::PoseStamped getPose(double t);

private:
  std::map<double, geometry_msgs::PoseStamped> trajectory;
  std::vector<double> keys;

  bool comparePoses(geometry_msgs::PoseStamped p1, geometry_msgs::PoseStamped p2);
};

#endif  // WAYPOINT_PLANNER_HUMAN_TRAJECTORY_H_
