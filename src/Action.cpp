#include "waypoint_planner/Action.h"

const uint8_t Action::OBSERVE = 0;
const uint8_t Action::MOVE = 1;

using std::vector;

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

void Action::duration(geometry_msgs::PointStamped s1, geometry_msgs::PointStamped s2, vector<double> &durations,
    vector<double> &probabilities)
{
  if (action_type == OBSERVE)
  {
    durations.push_back(1.0);
    probabilities.push_back(1.0);
  }
  else
  {
    // planning + execution time calculation, assuming the robot moves at .5 m/s
    double base_planning_time = 5.0;
    double base_travel_time = 0.5 * sqrt(pow(s1.point.x - s2.point.x, 2) + pow(s1.point.y - s2.point.y, 2) +
                                         pow(s1.point.z - s2.point.z, 2));
    double discretization = 0.05;
    double scale_lower_bound = 0.75;
    double scale_upper_bound = 1.25;
    int steps = static_cast<int>((scale_lower_bound - scale_upper_bound)/discretization) + 1;
    double uniform_probability = 1.0/static_cast<double>(steps);
    for (int i = 0; i < steps; i ++)
    {
      durations.push_back(base_planning_time + (scale_lower_bound + i*discretization)*base_travel_time);
      probabilities.push_back(uniform_probability);
    }
  }
}
