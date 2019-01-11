#include "waypoint_planner/Action.h"

const uint8_t Action::OBSERVE = 0;
const uint8_t Action::MOVE = 1;

using std::vector;

Action::Action(uint8_t action_type)
{
  this->action_type = action_type;
}

Action::Action(uint8_t action_type, geometry_msgs::Point action_goal)
{
  this->action_type = action_type;
  this->action_goal = action_goal;
}

void Action::setActionType(uint8_t action_type)
{
  this->action_type = action_type;
}

void Action::setGoal(geometry_msgs::Point action_goal)
{
  this->action_goal = action_goal;
}

uint8_t Action::actionType()
{
  return action_type;
}

geometry_msgs::Point Action::actionGoal()
{
  return action_goal;
}

void Action::duration(geometry_msgs::Point s1, geometry_msgs::Point s2, vector<double> &durations,
    vector<double> &probabilities)
{
  if (action_type == OBSERVE)
  {
    durations.push_back(1.5);
    probabilities.push_back(1.0);
  }
  else
  {
    // planning + execution time calculation, assuming the robot moves at .5 m/s
    double base_planning_time = 3.0;
    double base_travel_time = 2.0 * sqrt(pow(s1.x - s2.x, 2) + pow(s1.y - s2.y, 2) + pow(s1.z - s2.z, 2));
    double discretization = 0.05;
    double scale_lower_bound = 1.0;
    double scale_upper_bound = 2.0;
    int steps = static_cast<int>((scale_upper_bound - scale_lower_bound)/discretization) + 1;
    double uniform_probability = 1.0/static_cast<double>(steps);
    for (int i = 0; i < steps; i ++)
    {
      durations.push_back(base_planning_time + (scale_lower_bound + i*discretization)*base_travel_time);
      probabilities.push_back(uniform_probability);
    }
  }
}
