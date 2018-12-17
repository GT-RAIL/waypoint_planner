#include "waypoint_planner/State.h"

State::State()
{

}

State::State(geometry_msgs::Point robot_pose)
{
  this->robot_pose = robot_pose;
}

State::State(geometry_msgs::Point robot_pose, geometry_msgs::Pose human_pose)
{
  this->robot_pose = robot_pose;
  this->human_pose = human_pose;
}

geometry_msgs::Point State::robotPose()
{
  return robot_pose;
}

geometry_msgs::Pose State::humanPose()
{
  return human_pose;
}
