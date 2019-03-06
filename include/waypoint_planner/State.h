#ifndef WAYPOINT_PLANNER_STATE_H_
#define WAYPOINT_PLANNER_STATE_H_

// ROS
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>

class State
{
public:
    State();

    State(geometry_msgs::Point robot_pose, bool perched);

    State(geometry_msgs::Point robot_pose, bool perched, geometry_msgs::Pose human_pose);

    void setRobotPose(geometry_msgs::Point pose);

    void setPerched(bool perched);

    void setHumanPose(geometry_msgs::Pose pose);

    geometry_msgs::Point robotPose();

    bool isPerched();

    geometry_msgs::Pose humanPose();

private:
  geometry_msgs::Point robot_pose;
  bool perched;
  geometry_msgs::Pose human_pose;
};

#endif  // WAYPOINT_PLANNER_STATE_H_
