#ifndef WAYPOINT_PLANNER_HUMAN_TRAJECTORY_H_
#define WAYPOINT_PLANNER_HUMAN_TRAJECTORY_H_

// ROS
#include <geometry_msgs/Pose.h>
#include <tf2_bullet/tf2_bullet.h>

class HumanTrajectory
{
public:
    HumanTrajectory();

    void addPose(double time, geometry_msgs::Pose pose);

    void randomizeTrajectory(double arc_trans=0.8, double arc_rot=0.5, double key_trans=0.2, double key_rot=0.15);

    void interpolate(double step);

    void sortKeys();

    geometry_msgs::Pose getPose(double t);

private:
  std::map<double, geometry_msgs::Pose> trajectory;
  std::vector<double> keys;

  bool comparePoses(geometry_msgs::Pose p1, geometry_msgs::Pose p2);
};

#endif  // WAYPOINT_PLANNER_HUMAN_TRAJECTORY_H_
