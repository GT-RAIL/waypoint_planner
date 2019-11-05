#include "waypoint_planner/ReactiveBaseline.h"

using std::pair;
using std::string;
using std::vector;

ReactiveBaseline::ReactiveBaseline(string waypoint_file_name, vector<double> weights)
{
  linearization_weights = weights;

  loadWaypoints(waypoint_file_name);
  SMDPFunctions::initializeActions(waypoints, actions);

  perch_states.clear();
  for (size_t i = 0; i < waypoints.size(); i ++)
  {
    perch_states.emplace_back(PerchState(waypoints[i], false));
    perch_states.emplace_back(PerchState(waypoints[i], true));
  }

  // default weights
  if (linearization_weights.size() == 0)
  {
    for (size_t i = 0; i < 4; i ++)
    {
      linearization_weights.push_back(0.25);
    }
  }

  default_human_dims.x = 0.5;
  default_human_dims.y = 0.4;
  default_human_dims.z = 1.4;
}

void ReactiveBaseline::setWeights(vector<double> weights)
{
  linearization_weights = weights;
}

void ReactiveBaseline::setTrajectory(std::vector<HumanTrajectory> trajectories)
{
  this->trajectories = trajectories;
}

// Deprecated with new trajectory sampling API (trajectories should be passed in instead)
void ReactiveBaseline::loadTrajectory(std::string file_name)
{
  string trajectory_file_path = ros::package::getPath("waypoint_planner") + "/config/" + file_name;

  trajectories.resize(1);
  trajectories[0] = EnvironmentSetup::readHumanTrajectory(trajectory_file_path);
}

void ReactiveBaseline::loadWaypoints(string file_name)
{
  string waypoint_file_path = ros::package::getPath("waypoint_planner") + "/config/" + file_name;
  EnvironmentSetup::readWaypoints(waypoint_file_path, waypoints);
}

Action ReactiveBaseline::getAction(PerchState s, double t)
{
  double best_u = std::numeric_limits<double>::lowest();
  Action best_action(Action::OBSERVE);
  for (PerchState ps : perch_states)
  {
    double u = 0;
    for (HumanTrajectory traj : trajectories)
    {
      u += linearization_weights[0]*RewardsAndCosts::reward_recognition(traj.getPose(t), default_human_dims, ps.waypoint)
          + linearization_weights[1]*RewardsAndCosts::cost_collision(traj.getPose(t), default_human_dims, ps.waypoint)
          + linearization_weights[2]*RewardsAndCosts::cost_intrusion(traj.getPose(t), ps.waypoint, ps.perched);
      if (s.perched && !ps.perched)
      {
        u += linearization_weights[3]*RewardsAndCosts::cost_power(s.perched, Action(Action::UNPERCH));
      }
      else if (!s.perched && ps.perched)
      {
        u += linearization_weights[3]*RewardsAndCosts::cost_power(s.perched, Action(Action::PERCH));
      }

      if (s.waypoint.x != ps.waypoint.x || s.waypoint.y != ps.waypoint.y || s.waypoint.z != ps.waypoint.z)
      {
        u += linearization_weights[3]*RewardsAndCosts::cost_power(false, Action(Action::MOVE, ps.waypoint));
      }
    }

    if (u > best_u)
    {
      best_u = u;
      if (s.waypoint.x == ps.waypoint.x && s.waypoint.y == ps.waypoint.y && s.waypoint.z == ps.waypoint.z)
      {
        if (s.perched == ps.perched)
        {
          Action observe(Action::OBSERVE);
          best_action = observe;
        }
        else if (!s.perched)
        {
          Action perch(Action::PERCH);
          best_action = perch;
        }
        else
        {
          Action unperch(Action::UNPERCH);
          best_action = unperch;
        }
      }
      else
      {
        if (s.perched)
        {
          Action unperch(Action::UNPERCH);
          best_action = unperch;
        }
        else
        {
          Action move(Action::MOVE, ps.waypoint);
          best_action = move;
        }
      }
    }
  }

  return best_action;
}
