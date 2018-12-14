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
//    std::cout << "Reward for observe at waypoint " << i << ": " << utility_map[i][t_end] << std::endl;
  }

  // perform backwards induction for policy
  for (long t = static_cast<long>(t_end - 1); t >= 0; t --)
  {
//    std::cout << "TIMESTEP: " << t << std::endl;
    for (size_t i = 0; i < waypoints.size(); i ++)
    {
//      std::cout << "Waypoint #: " << i << std::endl;

      State s(waypoints[i], trajectory.getPose(t * time_step));

      Action best_a(Action::OBSERVE);
      double best_u = std::numeric_limits<double>::lowest();
      for (Action a : actions)
      {
        if (a.actionGoal().point.x == s.robotPose().point.x
            && a.actionGoal().point.y == s.robotPose().point.y
            && a.actionGoal().point.z == s.robotPose().point.z)
          continue;

//        if (i < 3)
//        {
//          if (a.actionType() == Action::MOVE)
//          {
//            std::cout << "\tAction: Move, waypoint: " << a.actionGoal().point.x << ", " << a.actionGoal().point.y
//                      << ", " << a.actionGoal().point.z << std::endl;
//          }
//          else
//          {
//            std::cout << "\tAction: Observe" << std::endl;
//          }
//        }

        double u = reward(s, a);
        vector<geometry_msgs::PointStamped> s_primes;
        vector<double> transition_probabilities;
        transition_model(s.robotPose(), a, s_primes, transition_probabilities);
        for (size_t j = 0; j < s_primes.size(); j ++)
        {
          double u2 = 0;
          vector<double> dts;
          vector<double> dt_probabilities;
          a.duration(s.robotPose(), s_primes[j], dts, dt_probabilities);
          size_t new_waypoint_index = waypoint_to_index(s_primes[j]);
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

//        if (i < 3)
//          std::cout << "\t\tUtility: " << u << std::endl;
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
  vector<geometry_msgs::PointStamped> s_primes;
  vector<double> transition_probabilities;
  transition_model(s.robotPose(), a, s_primes, transition_probabilities);

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
        r2 += probabilities[j] * RewardsAndCosts::reward_recognition(s.humanPose(), default_human_dims, s.robotPose())
             * durations[j];
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

void SMDPSolver::transition_model(geometry_msgs::PointStamped s, Action a, vector<geometry_msgs::PointStamped> &s_primes,
    vector<double> &probabilities)
{
  geometry_msgs::PointStamped s_prime;
  if (a.actionType() == Action::OBSERVE)
  {
    s_prime.header.frame_id = s.header.frame_id;
    s_prime.point.x = s.point.x;
    s_prime.point.y = s.point.y;
    s_prime.point.z = s.point.z;
  }
  else
  {
    s_prime.header.frame_id = a.actionGoal().header.frame_id;
    s_prime.point.x = a.actionGoal().point.x;
    s_prime.point.y = a.actionGoal().point.y;
    s_prime.point.z = a.actionGoal().point.z;
  }

  s_primes.push_back(s_prime);
  probabilities.push_back(1.0);
}

Action SMDPSolver::get_action(geometry_msgs::PointStamped s, double t)
{
  return get_action(s, static_cast<size_t>(t / time_step));
}

Action SMDPSolver::get_action(geometry_msgs::PointStamped s, size_t t)
{
  return action_map[waypoint_to_index(s)][t];
}

size_t SMDPSolver::waypoint_to_index(geometry_msgs::PointStamped w)
{
  for (size_t i = 0; i < waypoints.size(); i ++)
  {
    if (waypoints[i].point.x == w.point.x && waypoints[i].point.y == w.point.y && waypoints[i].point.z == w.point.z)
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
