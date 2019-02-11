#include "waypoint_planner/State.h"

State::State()
{

}

State::State(geometry_msgs::Point robot_pose, bool perched)
{
  this->robot_pose = robot_pose;
  this->perched = perched;
}

State::State(geometry_msgs::Point robot_pose, bool perched, geometry_msgs::Pose human_pose)
{
  this->robot_pose = robot_pose;
  this->perched = perched;
  this->human_pose = human_pose;
}

void State::setPerched(bool perched)
{
  perched = this->perched;
}

geometry_msgs::Point State::robotPose()
{
  return robot_pose;
}

bool State::isPerched()
{
  return perched;
}

geometry_msgs::Pose State::humanPose()
{
  return human_pose;
}
