#include "waypoint_planner/MCTSSolver.h"

using std::string;
using std::vector;
using std::cout;
using std::endl;
using std::max;
using std::min;

MCTSSolver::MCTSSolver(double horizon, double step, string trajectory_file_name, string waypoint_file_name, vector<double> constraints, double timeout_sec, size_t max_time_step_search_depth, double exploration_constant)
{
  time_horizon = horizon;
  time_step = step;
  t_end = static_cast<size_t>(time_horizon / time_step);

  cout << "Setting time scale to end at time index: " << t_end << endl;

  loadTrajectory(move(trajectory_file_name));
  loadWaypoints(move(waypoint_file_name));
  cout << "Loaded " << waypoints.size() << " waypoints." << endl;

  // initialize list of states
  for (size_t i = 0; i < waypoints.size(); i ++)
  {
    for (size_t j = 0; j <= t_end; j ++)
    {
      states.emplace_back(StateWithTime(i, j));
    }
  }

  cout << "Initialized " << states.size() << " states." << endl;

  SMDPFunctions::initializeActions(waypoints, actions);

  cout << "Initialized " << actions.size() << " actions." << endl;

  cout << "Constructing (s(t), a) index list..." << endl;

  num_variables = 0;
  index_map.resize(waypoints.size());
  for (size_t i = 0; i < waypoints.size(); i ++)
  {
    index_map[i].resize(t_end + 1);
    for (size_t j = 0; j <= t_end; j ++)
    {
      index_map[i][j].resize(actions.size());
      for (size_t k = 0; k < actions.size(); k ++)
      {
        if (isValidAction(i, k))
        {
          index_map[i][j][k] = num_variables;
          num_variables ++;
        }
        else
        {
          index_map[i][j][k] = std::numeric_limits<size_t>::max();
        }
      }
    }
  }

  default_human_dims.x = 0.5;
  default_human_dims.y = 0.4;
  default_human_dims.z = 1.4;

  num_costs = constraints.size();
  cost_constraints.resize(num_costs);
  for (size_t i = 0; i < constraints.size(); i ++)
  {
    cost_constraints[i] = constraints[i];
  }

  lambda.resize(num_costs);
  timeout = ros::Duration(timeout_sec);
  max_search_time_step_dst = max_time_step_search_depth;

  this->exploration_constant = exploration_constant;

  VC_s.resize(num_costs);
  QC_sa.resize(num_costs);
}

size_t MCTSSolver::search(size_t waypoint_index, size_t time_step)
{
  return search(StateWithTime(waypoint_index, time_step));
}

size_t MCTSSolver::search(StateWithTime s0)
{
  //initialize parameters
  for (auto lam : lambda)
  {
    lam = static_cast<double>(rand())/RAND_MAX;
  }
  learning_rate = 1.0;
  max_search_time_step = min(s0.time_index + max_search_time_step_dst, t_end);

  ros::Time start_time = ros::Time::now();
  ros::Time end_time = start_time + timeout;
  while (ros::Time::now() < end_time)
  {
    simulate(s0);
    size_t a_id = greedyPolicy(s0, 0);
    for (size_t i = 0; i < lambda.size(); i ++)
    {
      // update lambdas
      lambda[i] += learning_rate*(QC_sa[i][getIndexSA(s0.waypoint_id, s0.time_index, a_id)] - cost_constraints[i]);

      // clip lambdas to [0, infinity)
      // NOTE: if using discounts, this needs to be updated to R_max/(tau*(1-gamma))
      lambda[i] = max(0.0, lambda[i]);
    }

    // update learning rate
    learning_rate = 1.0/(1 + 10*(ros::Time::now() - start_time).toSec());
  }

  return greedyPolicy(s0, 0);
}

vector<double> MCTSSolver::simulate(StateWithTime s)
{
  // simulation rollout termination
  if (s.time_index >= max_search_time_step)
  {
    return vector<double>(num_costs + 1);  // size: 1 + costs; order: {R, C0, C1, ..., Cn}
  }

  // if we're seeing a state for the first time
  size_t s_index = getIndexS(s);
  if (N_s.count(getIndexS(s)) == 0)
  {
    N_s[s_index] = 0;
    for (auto VCi_s : VC_s)
    {
      VCi_s[s_index] = 0;
    }

    for (size_t i = 0; i < actions.size(); i ++)
    {
      if (isValidAction(s.waypoint_id, i))
      {
        size_t sa_index = getIndexSA(s.waypoint_id, s.time_index, i);
        N_sa[sa_index] = 0;
        QR_sa[sa_index] = 0;
        for (auto QCi_sa : QC_sa)
        {
          QCi_sa[sa_index] = 0;
        }
      }
    }

    return rollout(s);
  }

  // select an action (with an exploration tradeoff)
  size_t a_id = greedyPolicy(s, exploration_constant);

  // simulate the selected action, calculate costs and rewards
  // NOTE: this currently doesn't use the state transition function, as actions deterministically move to new states
  vector<double> total_costs(num_costs + 1);  // size: 1 + costs; order: {R, C0, C1, ..., Cn}
  StateWithTime s_prime = simulate_action(s, actions[a_id], total_costs);

  vector<double> costs_to_go = simulate(s_prime);
  for (size_t i = 0; i < total_costs.size(); i ++)
  {
    // NOTE: if using discount, this needs to be updated to gamma*costs_to_go[i]
    total_costs[i] += costs_to_go[i];
  }

  size_t sa_index = getIndexSA(s.waypoint_id, s.time_index, a_id);
  N_s[s_index] += 1;
  N_sa[sa_index] += 1;
  QR_sa[sa_index] += (total_costs[0] - QR_sa[sa_index])/N_sa[sa_index];
  for (size_t i = 0; i < VC_s.size(); i ++)
  {
    VC_s[i][s_index] += (total_costs[i + 1] - VC_s[i][s_index])/N_s[s_index];
    QC_sa[i][sa_index] += (total_costs[i + 1] - QC_sa[i][sa_index])/N_sa[sa_index];
  }

  return total_costs;
}

vector<double> MCTSSolver::rollout(StateWithTime s)
{
  // simulation rollout termination
  if (s.time_index >= max_search_time_step)
  {
    return vector<double>(num_costs + 1);  // size: 1 + costs; order: {R, C0, C1, ..., Cn}
  }

  // select an action (with a uniform(?) policy)
  size_t a_id = 0;  // observe action
  if (static_cast<double>(rand())/RAND_MAX > 0.5)  // take observe with probability 0.5, otherwise select uniformly
  {
    double p = 0.0;
    double increment = 1.0/actions.size();
    double selection = static_cast<double>(rand())/RAND_MAX;
    for (size_t i = 0; i < actions.size(); i ++)
    {
      p += increment;
      if (p > selection)
      {
        a_id = i;
        break;
      }
    }
  }

  // simulate the selected action, calculate costs and rewards
  // NOTE: this currently doesn't use the state transition function, as actions deterministically move to new states
  vector<double> total_costs(num_costs + 1);  // size: 1 + costs; order: {R, C0, C1, ..., Cn}
  StateWithTime s_prime = simulate_action(s, actions[a_id], total_costs);

  vector<double> costs_to_go = rollout(s_prime);
  for (size_t i = 0; i < total_costs.size(); i ++)
  {
    // NOTE: if using discount, this needs to be updated to gamma*costs_to_go[i]
    total_costs[i] += costs_to_go[i];
  }

  return total_costs;
}

size_t MCTSSolver::greedyPolicy(StateWithTime s, double kappa)
{
  size_t s_index = getIndexS(s);
  vector<size_t> best_actions;
  double best_q = -999999999;  // TODO: numeric_limits...
  for (size_t i = 0; i < actions.size(); i ++)
  {
    if (isValidAction(s.waypoint_id, i))
    {
      size_t sa_index = getIndexSA(s.waypoint_id, s.time_index, i);
      double q = QR_sa[sa_index];
      for (size_t j = 0; j < lambda.size(); j ++)
      {
        q -= lambda[j]*QC_sa[j][sa_index];
      }
      if (N_sa.count(sa_index) == 1 && N_sa[sa_index] > 0)
      {
        q += kappa*sqrt(log(N_s[s_index])/N_sa[sa_index]);
      }

      if (q > best_q)
      {
        best_actions.clear();
        best_actions.push_back(i);
        best_q = q;
      }
      else if (q == best_q)
      {
        best_actions.push_back(i);
      }
    }
  }

  // currently select uniformly as an approximation to avoid solving a convex optimization problem
  size_t best_action = best_actions[0];
  if (best_actions.size() > 1)
  {
    double p = 0;
    double increment = 1.0/best_actions.size();
    double selection = static_cast<double>(rand())/RAND_MAX;
    for (auto a : best_actions)
    {
      p += increment;
      if (p > selection)
      {
        best_action = a;
        break;
      }
    }
  }
  return best_action;
}

void MCTSSolver::setConstraints(vector<double> constraints)
{
  if (constraints.size() != cost_constraints.size())
  {
    cout << "New costs do not match the number of constraints!" << endl;
    cout << "Cost constraints were not updated." << endl;
    return;
  }
  for (size_t i = 0; i < constraints.size(); i ++)
  {
    cost_constraints[i] = constraints[i];
  }
}

StateWithTime MCTSSolver::simulate_action(StateWithTime s, Action a, vector<double> &result_costs)
{
  geometry_msgs::Point goal;
  if (a.actionType() == Action::MOVE)
  {
    goal = a.actionGoal();
  }
  else
  {
    goal = waypoints[s.waypoint_id];
  }

  vector<double> durations;
  vector<double> probabilities;
  a.duration(waypoints[s.waypoint_id], goal, durations, probabilities);
  double n = static_cast<double>(rand())/RAND_MAX;
  double duration = durations[0];
  double p = 0;
  for (size_t i = 0; i < probabilities.size(); i ++)
  {
    p += probabilities[i];
    if (p >= n)
    {
      duration = durations[i];
      break;
    }
  }

  StateWithTime s_prime(waypointToIndex(goal), static_cast<size_t>(ceil(s.time_index + duration/time_step)));

  // calculate rewards and costs and add them to the totals
  if (a.actionType() == Action::OBSERVE)
  {
    result_costs[0] = RewardsAndCosts::reward_recognition(trajectory.getPose(s.time_index * time_step),
        default_human_dims, waypoints[s.waypoint_id])*duration;
    result_costs[1] = RewardsAndCosts::cost_collision(trajectory.getPose(s.time_index * time_step), default_human_dims,
        waypoints[s.waypoint_id])*duration;
    result_costs[2] = RewardsAndCosts::cost_intrusion(trajectory.getPose(s.time_index * time_step),
        waypoints[s.waypoint_id])*duration;
  }

  return s_prime;
}

size_t MCTSSolver::getIndexSA(size_t waypoint_id, size_t t, size_t action_id)
{
  return index_map[waypoint_id][t][action_id];
}

size_t MCTSSolver::getIndexSA(size_t state_id, size_t action_id)
{
  return getIndexSA(states[state_id].waypoint_id, states[state_id].time_index, action_id);
}

size_t MCTSSolver::getIndexS(StateWithTime s)
{
  return getIndexS(s.waypoint_id, s.time_index);
}

size_t MCTSSolver::getIndexS(size_t waypoint_id, size_t t)
{
  return waypoint_id*(t_end + 1) + t;
}

void MCTSSolver::loadTrajectory(std::string file_name)
{
  string trajectory_file_path = ros::package::getPath("waypoint_planner") + "/config/" + file_name;

  trajectory = EnvironmentSetup::readHumanTrajectory(trajectory_file_path);
}

void MCTSSolver::loadWaypoints(string file_name)
{
  string waypoint_file_path = ros::package::getPath("waypoint_planner") + "/config/" + file_name;
  EnvironmentSetup::readWaypoints(waypoint_file_path, waypoints);
}

Action MCTSSolver::getAction(geometry_msgs::Point s, double t)
{
  return getAction(s, static_cast<size_t>(t / time_step));
}

Action MCTSSolver::getAction(geometry_msgs::Point s, size_t t)
{
  return actions[greedyPolicy(StateWithTime(waypointToIndex(s), t), 0)];
}

// TODO: better lookup (hash waypoints to indices maybe?)
size_t MCTSSolver::waypointToIndex(geometry_msgs::Point w)
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

double MCTSSolver::reward(size_t state_id, size_t action_id, uint8_t mode)
{
  return SMDPFunctions::reward(State(waypoints[states[state_id].waypoint_id],
                                     trajectory.getPose(states[state_id].time_index*time_step)),
                               actions[action_id], mode);
}

bool MCTSSolver::isValidAction(size_t waypoint_id, size_t action_id)
{
  return !(actions[action_id].actionType() == Action::MOVE
    && waypoints[waypoint_id].x == actions[action_id].actionGoal().x
    && waypoints[waypoint_id].y == actions[action_id].actionGoal().y
    && waypoints[waypoint_id].z == actions[action_id].actionGoal().z);
}
