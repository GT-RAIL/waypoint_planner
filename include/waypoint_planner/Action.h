#ifndef WAYPOINT_PLANNER_ACTION_H_
#define WAYPOINT_PLANNER_ACTION_H_

// ROS
#include <geometry_msgs/Point.h>

class Action
{
public:
    static const uint8_t OBSERVE;
    static const uint8_t MOVE;
    static const uint8_t PERCH;
    static const uint8_t UNPERCH;

    Action(uint8_t action_type);

    Action(uint8_t action_type, geometry_msgs::Point action_goal);

    void setActionType(uint8_t action_type);

    void setGoal(geometry_msgs::Point pose);

    uint8_t actionType();

    geometry_msgs::Point actionGoal();

    void duration(geometry_msgs::Point s1, geometry_msgs::Point s2, std::vector<double> &durations,
        std::vector<double> &probabilities);

private:
  uint8_t action_type;
  geometry_msgs::Point action_goal;
};

#endif  // WAYPOINT_PLANNER_ACTION_H_
