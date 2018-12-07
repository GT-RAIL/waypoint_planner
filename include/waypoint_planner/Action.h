#ifndef WAYPOINT_PLANNER_ACTION_H_
#define WAYPOINT_PLANNER_ACTION_H_

// ROS
#include <geometry_msgs/PointStamped.h>

class Action
{
public:
    static const uint8_t OBSERVE;
    static const uint8_t MOVE;

    Action(uint8_t action_type);

    Action(uint8_t action_type, geometry_msgs::PointStamped action_goal);

    void setActionType(uint8_t action_type);

    void setGoal(geometry_msgs::PointStamped pose);

    uint8_t actionType();

    geometry_msgs::PointStamped actionGoal();

private:
  uint8_t action_type;
  geometry_msgs::PointStamped action_goal;
};

#endif  // WAYPOINT_PLANNER_ACTION_H_
