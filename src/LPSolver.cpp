#include "waypoint_planner/LPSolver.h"

using std::string;
using std::vector;

LPSolver::LPSolver(double horizon, double step)
{
  lprec *lp;
  lp = make_lp(0, 2);

  time_horizon = horizon;
  time_step = step;

  default_human_dims.x = 0.5;
  default_human_dims.y = 0.4;
  default_human_dims.z = 1.4;
}

LPSolver::LPSolver(double horizon, double step, string trajectory_file_name, string waypoint_file_name)
{
  time_horizon = horizon;
  time_step = step;
  loadTrajectory(trajectory_file_name);
  loadWaypoints(waypoint_file_name);
  initializeActions();

  default_human_dims.x = 0.5;
  default_human_dims.y = 0.4;
  default_human_dims.z = 1.4;
}

void LPSolver::loadTrajectory(std::string file_name)
{
  string trajectory_file_path = ros::package::getPath("waypoint_planner") + "/config/" + file_name;

  trajectory = EnvironmentSetup::readHumanTrajectory(trajectory_file_path);
}

void LPSolver::loadWaypoints(string file_name)
{
  string waypoint_file_path = ros::package::getPath("waypoint_planner") + "/config/" + file_name;
  EnvironmentSetup::readWaypoints(waypoint_file_path, waypoints);
}

void LPSolver::initializeActions()
{
  actions.clear();
  actions.push_back(Action(Action::OBSERVE));
  for (auto& waypoint : waypoints)
  {
    actions.push_back(Action(Action::MOVE, waypoint));
  }
}

Action LPSolver::getAction(geometry_msgs::Point s, double t)
{
  // return getAction(s, static_cast<size_t>(t / time_step));
  return Action(0);
}

Action LPSolver::getAction(geometry_msgs::Point s, size_t t)
{
  // return action_map[waypointToIndex(s)][t];
  return Action(0);
}

// TODO: better lookup (hash waypionts to indices maybe?)
size_t LPSolver::waypointToIndex(geometry_msgs::Point w)
{
  for (size_t i = 0; i < waypoints.size(); i ++)
  {
    if (waypoints[i].x == w.x && waypoints[i].y == w.y && waypoints[i].z == w.z)
    {
      return i;
    }
  }
  return waypoints.size();  // error case, waypoint not found in list
}
