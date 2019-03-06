#include "waypoint_planner/PerchState.h"

PerchState::PerchState()
{
  perched = false;
}

PerchState::PerchState(geometry_msgs::Point waypoint)
{
  this->waypoint = waypoint;
}

PerchState::PerchState(geometry_msgs::Point waypoint, bool perched)
{
  this->waypoint = waypoint;
  this->perched = perched;
}
