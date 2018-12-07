#include "waypoint_planner/Action.h"

const uint8_t Action::OBSERVE = 0;
const uint8_t Action::MOVE = 1;

Action::Action(uint8_t action_type)
{
  this->action_type = action_type;
}

Action::Action(uint8_t action_type, geometry_msgs::PointStamped action_goal)
{
  this->action_type = action_type;
  this->action_goal = action_goal;
}

void Action::setActionType(uint8_t action_type)
{
  this->action_type = action_type;
}

void Action::setGoal(geometry_msgs::PointStamped action_goal)
{
  this->action_goal = action_goal;
}

uint8_t Action::actionType()
{
  return action_type;
}

geometry_msgs::PointStamped Action::actionGoal()
{
  return action_goal;
}
