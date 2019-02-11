#ifndef WAYPOINT_PLANNER_PERCH_STATE_H_
#define WAYPOINT_PLANNER_PERCH_STATE_H_

// ROS
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>

class PerchState
{
public:
    geometry_msgs::Point waypoint;
    bool perched;

    PerchState();

    PerchState(geometry_msgs::Point robot_pose);

    PerchState(geometry_msgs::Point robot_pose, bool perched);
};

#endif  // WAYPOINT_PLANNER_PERCH_STATE_H_
