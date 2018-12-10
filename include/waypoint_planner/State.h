#ifndef WAYPOINT_PLANNER_STATE_H_
#define WAYPOINT_PLANNER_STATE_H_

// ROS
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>

class State
{
public:
    State();

    State(geometry_msgs::PointStamped robot_pose);

    State(geometry_msgs::PointStamped robot_pose, geometry_msgs::PoseStamped human_pose);

    void setRobotPose(geometry_msgs::PointStamped pose);

    void setHumanPose(geometry_msgs::PoseStamped pose);

    geometry_msgs::PointStamped robotPose();

    geometry_msgs::PoseStamped humanPose();

private:
  geometry_msgs::PointStamped robot_pose;
  geometry_msgs::PoseStamped human_pose;
};

#endif  // WAYPOINT_PLANNER_STATE_H_
