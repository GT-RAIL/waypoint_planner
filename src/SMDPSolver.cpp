#include "waypoint_planner/SMDPSolver.h"

using std::pair;
using std::string;
using std::vector;

const uint8_t SMDPSolver::REWARD_ONLY = 0;
const uint8_t SMDPSolver::LINEARIZED_COST = 1;

SMDPSolver::SMDPSolver(double horizon, double step, uint8_t mode)
{
  this->mode = mode;
  time_horizon = horizon;
  time_step = step;

  // default weights
  if (this->mode == LINEARIZED_COST)
  {
    for (size_t i = 0; i < 3; i ++)
    {
      linearization_weights.push_back(0.333333);
    }
  }

  default_human_dims.x = 0.5;
  default_human_dims.y = 0.4;
  default_human_dims.z = 1.4;
}

SMDPSolver::SMDPSolver(double horizon, double step, uint8_t mode, string trajectory_file_name,
    string waypoint_file_name, vector<double> weights)
{
  this->mode = mode;
  linearization_weights = weights;
  time_horizon = horizon;
  time_step = step;
  loadTrajectory(trajectory_file_name);
  loadWaypoints(waypoint_file_name);
  initializeActions();

  // default weights
  if (linearization_weights.size() == 0 && this->mode == LINEARIZED_COST)
  {
    for (size_t i = 0; i < 3; i ++)
    {
      linearization_weights.push_back(0.333333);
    }
  }

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
    if (mode == REWARD_ONLY)
    {
      utility_map[i][t_end] = RewardsAndCosts::reward_recognition(trajectory.getPose(t_end * time_step),
                                                                  default_human_dims, waypoints[i]);
    }
    else
    {
      utility_map[i][t_end] = linearizedCost(trajectory.getPose(t_end * time_step), default_human_dims, waypoints[i]);
    }
  }

  // perform backwards induction for policy
  for (long t = static_cast<long>(t_end - 1); t >= 0; t --)
  {
    for (size_t i = 0; i < waypoints.size(); i ++)
    {
      State s(waypoints[i], trajectory.getPose(t * time_step));

      Action best_a(Action::OBSERVE);
      double best_u = std::numeric_limits<double>::lowest();
      for (Action a : actions)
      {
        if (a.actionGoal().x == s.robotPose().x
            && a.actionGoal().y == s.robotPose().y
            && a.actionGoal().z == s.robotPose().z)
          continue;

        double u = reward(s, a);
        vector<geometry_msgs::Point> s_primes;
        vector<double> transition_probabilities;
        transitionModel(s.robotPose(), a, s_primes, transition_probabilities);
        for (size_t j = 0; j < s_primes.size(); j ++)
        {
          double u2 = 0;
          vector<double> dts;
          vector<double> dt_probabilities;
          a.duration(s.robotPose(), s_primes[j], dts, dt_probabilities);
          size_t new_waypoint_index = waypointToIndex(s_primes[j]);
          for (size_t k = 0; k < dts.size(); k ++)
          {
            // convert time index to time to updated time index
            size_t t_prime = static_cast<size_t>((t*time_step + dts[k]) / time_step);
            if (t_prime == t)  // round up to check reward at next decision point
            {
              t_prime ++;
            }

            if (t_prime <= t_end)  // make sure we don't exceed the finite horizon
            {
              u2 += dt_probabilities[k]*utility_map[new_waypoint_index][t_prime];
            }
          }
          u += transition_probabilities[j]*u2;
        }

        // update max utility and argmax action
        if (u > best_u)
        {
          best_u = u;
          best_a = a;
        }
      }

      utility_map[i][t] = best_u;
      action_map[i][t] = best_a;
    }
  }

  std::cout << "Backwards induction complete." << std::endl;
}

double SMDPSolver::reward(State s, Action a)
{
  vector<geometry_msgs::Point> s_primes;
  vector<double> transition_probabilities;
  transitionModel(s.robotPose(), a, s_primes, transition_probabilities);

  double r = 0;

  for (size_t i = 0; i < s_primes.size(); i ++)
  {
    vector<double> durations;
    vector<double> probabilities;

    a.duration(s.robotPose(), s_primes[i], durations, probabilities);

    double r2 = 0;
    for (size_t j = 0; j < durations.size(); j++)
    {
      if (a.actionType() == Action::OBSERVE)
      {
        if (mode == REWARD_ONLY)
        {
          r2 += probabilities[j] * RewardsAndCosts::reward_recognition(s.humanPose(), default_human_dims, s.robotPose())
                * durations[j];
        }
        else
        {
          r2 += probabilities[j] * linearizedCost(s.humanPose(), default_human_dims, s.robotPose()) * durations[j];
        }
      }
      else
      {
        r2 += probabilities[j] * 0 * durations[j];  //TODO: reward/cost while in transit (currently set to 0)
      }
    }

    r += transition_probabilities[i]*r2;
  }

  return r;
}

double SMDPSolver::linearizedCost(geometry_msgs::Pose h, geometry_msgs::Vector3 human_dims,
    geometry_msgs::Point r)
{
  return linearization_weights[0]*RewardsAndCosts::reward_recognition(h, human_dims, r)
       - linearization_weights[1]*RewardsAndCosts::cost_collision(h, human_dims, r)
       - linearization_weights[2]*RewardsAndCosts::cost_intrusion(h, r);
}

void SMDPSolver::transitionModel(geometry_msgs::Point s, Action a, vector<geometry_msgs::Point> &s_primes,
    vector<double> &probabilities)
{
  geometry_msgs::Point s_prime;
  if (a.actionType() == Action::OBSERVE)
  {
    s_prime.x = s.x;
    s_prime.y = s.y;
    s_prime.z = s.z;
  }
  else
  {
    s_prime.x = a.actionGoal().x;
    s_prime.y = a.actionGoal().y;
    s_prime.z = a.actionGoal().z;
  }

  s_primes.push_back(s_prime);
  probabilities.push_back(1.0);
}

Action SMDPSolver::getAction(geometry_msgs::Point s, double t)
{
  return getAction(s, static_cast<size_t>(t / time_step));
}

Action SMDPSolver::getAction(geometry_msgs::Point s, size_t t)
{
  return action_map[waypointToIndex(s)][t];
}

// TODO: better lookup (hash waypionts to indices maybe?)
size_t SMDPSolver::waypointToIndex(geometry_msgs::Point w)
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

//int main(int argc, char **argv)
//{
//  SMDPSolver solver(155, 1, "iss_trajectory.yaml", "iss_waypoints.csv");
//  solver.backwardsInduction();
//}
