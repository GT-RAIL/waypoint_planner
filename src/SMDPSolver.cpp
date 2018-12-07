#include "waypoint_planner/SMDPSolver.h"

using std::pair;
using std::string;
using std::vector;

SMDPSolver::SMDPSolver(double horizon, double step)
{
  time_horizon = horizon;
  time_step = step;

  default_human_dims.x = 0.5;
  default_human_dims.y = 0.4;
  default_human_dims.z = 1.4;
}

SMDPSolver::SMDPSolver(double horizon, double step, std::string trajectory_file_name, std::string waypoint_file_name)
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

void SMDPSolver::loadTrajectory(std::string file_name)
{
  string trajectory_file_path = ros::package::getPath("waypoint_planner") + "/config/" + file_name;

  trajectory = EnvironmentSetup::readHumanTrajectory(trajectory_file_path);
}

void SMDPSolver::loadWaypoints(string file_name)
{
  string waypoint_file_path = ros::package::getPath("waypoint_planner") + "/config/" + file_name;
  EnvironmentSetup::readWaypoints(waypoint_file_path, waypoints);
}

void SMDPSolver::initializeActions()
{
  actions.clear();
  actions.push_back(Action(Action::OBSERVE));
  for (auto& waypoint : waypoints)
  {
    actions.push_back(Action(Action::MOVE, waypoint));
  }
}

void SMDPSolver::backwardsInduction()
{
  auto t_end = static_cast<size_t>(time_horizon / time_step);

  // initialize optimal utility and action lists
  utility_map.clear();
  action_map.clear();
  vector<double> u_init(t_end + 1, 0.0);
  vector<Action> a_init(t_end + 1, Action(Action::OBSERVE));
  for (size_t i = 0; i < waypoints.size(); i ++)
  {
    utility_map.push_back(u_init);
    action_map.push_back(a_init);
  }

  // initialize final time step utilities
  for (size_t i = 0; i < waypoints.size(); i ++)
  {
    utility_map[i][t_end] = RewardsAndCosts::reward_recognition(trajectory.getPose(t_end*time_step),
        default_human_dims, waypoints[i]);
  }

  // perform backwards induction for policy
  for (long t = static_cast<long>(t_end - 1); t >= 0; t --)
  {
    
  }
}

int main(int argc, char **argv)
{
  SMDPSolver solver(155, 1, "iss_trajectory.yaml", "iss_waypoints.csv");
  solver.backwardsInduction();
}
