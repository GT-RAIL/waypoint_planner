#include "waypoint_planner/State.h"

State::State()
{

}

State::State(geometry_msgs::PoseStamped robot_pose)
{
  this->robot_pose = robot_pose;
}

State::State(geometry_msgs::PoseStamped robot_pose, geometry_msgs::PoseStamped human_pose)
{
  this->robot_pose = robot_pose;
  this->human_pose = human_pose;
}

geometry_msgs::PoseStamped State::robotPose()
{
  return robot_pose;
}

geometry_msgs::PoseStamped State::humanPose()
{
  return human_pose;
}
