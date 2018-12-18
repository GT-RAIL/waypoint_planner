#include "waypoint_planner/LPSolver.h"

using std::string;
using std::vector;

LPSolver::LPSolver(double horizon, double step, string trajectory_file_name, string waypoint_file_name)
{
  time_horizon = horizon;
  time_step = step;
  t_end = static_cast<size_t>(time_horizon / time_step);

  loadTrajectory(move(trajectory_file_name));
  loadWaypoints(move(waypoint_file_name));

  // initialize list of states
  for (size_t i = 0; i < waypoints.size(); i ++)
  {
    for (size_t j = 0; j <= t_end; j ++)
    {
      states.emplace_back(StateWithTime(i, j));
    }
  }

  SMDPFunctions::initializeActions(waypoints, actions);

  default_human_dims.x = 0.5;
  default_human_dims.y = 0.4;
  default_human_dims.z = 1.4;
}

void LPSolver::constructModel()
{
  // construct lp with |S_t|*|A| variables
  int num_variables = states.size() * actions.size();
  lp = make_lp(0, num_variables);

  // define objective function
  REAL row[1 + num_variables];
  for (size_t i = 0; i < states.size(); i++)
  {
    for (size_t j = 0; j < actions.size(); j ++)
    {
      // note: lp_solve API uses 1-indexing, so all row indices have +1 at the end
      row[i*actions.size() + j + 1] = reward(i, j, SMDPFunctions::REWARD);
    }
  }
  set_obj_fn(lp, row);

  // define the constraints row-by-row

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

Action LPSolver::getAction(geometry_msgs::Point s, double t)
{
  // return getAction(s, static_cast<size_t>(t / time_step));
  return {0};
}

Action LPSolver::getAction(geometry_msgs::Point s, size_t t)
{
  // return action_map[waypointToIndex(s)][t];
  return {0};
}

// TODO: better lookup (hash waypoints to indices maybe?)
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

double LPSolver::reward(size_t state_id, size_t action_id, uint8_t mode)
{
  return SMDPFunctions::reward(State(waypoints[states[state_id].waypoint_id],
                                     trajectory.getPose(states[state_id].time_index*time_step)),
                               actions[action_id], mode);
}

StateWithTime::StateWithTime(size_t waypoint_id, size_t time_index)
{
  this->waypoint_id = waypoint_id;
  this->time_index = time_index;
}

StateWithTime StateWithTime::indexToState(size_t i, size_t max_time_steps)
{
  return {static_cast<size_t>(floor(static_cast<double>(i)/max_time_steps)), i%max_time_steps};
}

size_t StateWithTime::stateToIndex(StateWithTime state, size_t max_time_steps)
{
  return stateToIndex(state.waypoint_id, state.time_index, max_time_steps);
}

size_t StateWithTime::stateToIndex(size_t waypoint_id, size_t time_index, size_t max_time_steps)
{
  return waypoint_id*max_time_steps + time_index;
}
