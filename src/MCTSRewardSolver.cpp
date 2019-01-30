#include "waypoint_planner/MCTSRewardSolver.h"

using std::string;
using std::vector;
using std::cout;
using std::endl;
using std::max;
using std::min;

MCTSRewardSolver::MCTSRewardSolver(double horizon, double step, string trajectory_file_name, string waypoint_file_name, vector<double> constraints, double timeout_sec, size_t max_time_step_search_depth, double exploration_constant)
{
  time_horizon = horizon;
  time_step = step;
  t_end = static_cast<size_t>(time_horizon / time_step);

  num_costs = constraints.size();
  cost_constraints.resize(num_costs);
  for (size_t i = 0; i < constraints.size(); i ++)
  {
    cost_constraints[i] = constraints[i];
  }

  cout << "Setting time scale to end at time index: " << t_end << endl;

  loadTrajectory(move(trajectory_file_name));
  loadWaypoints(move(waypoint_file_name));
  cout << "Loaded " << waypoints.size() << " waypoints." << endl;

  // initialize list of states, waypoint hashes
  for (size_t i = 0; i < waypoints.size(); i ++)
  {
    waypoint_index_map[waypointHash(waypoints[i])] = i;
    for (size_t j = 0; j <= t_end; j ++)
    {
      states.emplace_back(StateWithTime(i, j));
      N_s[getIndexS(i, j)] = 0;
    }
  }

  cout << "Initialized " << states.size() << " states, and initialized MCTS state-based containers." << endl;

  SMDPFunctions::initializeActions(waypoints, actions);

  cout << "Initialized " << actions.size() << " actions." << endl;

  cout << "Constructing (s(t), a) index list and initializing MCTS state-action-based containers..." << endl;

  QC_sa.resize(num_costs);

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
          N_sa[num_variables] = 0;
          QR_sa[num_variables] = 0;
          for (size_t l = 0; l < QC_sa.size(); l++)
          {
            QC_sa[l][num_variables] = 0;
          }
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

  timeout = ros::Duration(timeout_sec);
  max_search_time_step_dst = max_time_step_search_depth;

  this->exploration_constant = exploration_constant;
}

Action MCTSRewardSolver::search(geometry_msgs::Point w, double t)
{
  return search(waypointToIndex(w), static_cast<size_t>(t / time_step));
}

Action MCTSRewardSolver::search(size_t waypoint_index, size_t time_step)
{
  return search(StateWithTime(waypoint_index, time_step));
}

Action MCTSRewardSolver::search(StateWithTime s0)
{
  max_search_time_step = min(s0.time_index + max_search_time_step_dst, t_end);

//  // TODO: temporary testing; reset everything each search step
//  for (size_t i = 0; i < waypoints.size(); i ++)
//  {
//    for (size_t j = 0; j <= t_end; j ++)
//    {
//      N_s[getIndexS(i, j)] = 0;
//      for (size_t k = 0; k < VC_s.size(); k ++)
//      {
//        VC_s[k][getIndexS(i, j)] = 0;
//      }
//    }
//  }
//  num_variables = 0;
//  for (size_t i = 0; i < waypoints.size(); i ++)
//  {
//    for (size_t j = 0; j <= t_end; j ++)
//    {
//      for (size_t k = 0; k < actions.size(); k ++)
//      {
//        if (isValidAction(i, k))
//        {
//          N_sa[num_variables] = 0;
//          QR_sa[num_variables] = 0;
//          for (size_t l = 0; l < QC_sa.size(); l++)
//          {
//            QC_sa[l][num_variables] = 0;
//          }
//          num_variables ++;
//        }
//      }
//    }
//  }

  ros::Time start_time = ros::Time::now();
  ros::Time end_time = start_time + timeout;
  size_t iterations = 0;
  while (ros::Time::now() < end_time)
  {
    simulate(s0);
    iterations ++;
  }

  cout << "MCTS performed " << iterations << " rollouts over " << (ros::Time::now() - start_time).toSec() <<
    " seconds." << endl;

  size_t best_a_id = greedyPolicy(s0, 0, true);

  cout << "Action selected: " << best_a_id << endl;
  cout << "\tExpected results: " << endl;
  for (size_t i = 0; i < actions.size(); i ++)
  {
    if (isValidAction(s0.waypoint_id, i))
    {
      size_t sa_index = getIndexSA(s0.waypoint_id, s0.time_index, i);
      cout << "\tAction " << i << ": QR(s,a)=" << QR_sa[sa_index] << ", QC0(s,a)=" << QC_sa[0][sa_index] << ", QC1(s,a)=" << QC_sa[1][sa_index] << "; N(s,a)=" << N_sa[sa_index] << endl;
    }
  }

  return actions[best_a_id];
}

vector<double> MCTSRewardSolver::simulate(StateWithTime s)
{
  // simulation rollout termination
  if (s.time_index >= max_search_time_step)
  {
    return vector<double>(num_costs + 1);  // size: 1 + costs; order: {R, C0, C1, ..., Cn}
  }

  // if we're seeing a state for the first time
  size_t s_index = getIndexS(s);
//  if (N_s[getIndexS(s)] == 0)
//  {
//    N_s[s_index] = 1;
//    // this is now pre-initialized for speed...
////    for (auto VCi_s : VC_s)
////    {
////      VCi_s[s_index] = 0;
////    }
//
//    for (size_t i = 0; i < actions.size(); i ++)
//    {
//      if (isValidAction(s.waypoint_id, i))
//      {
////        size_t sa_index = getIndexSA(s.waypoint_id, s.time_index, i);
//        N_sa[getIndexSA(s.waypoint_id, s.time_index, i)] = 1;
//        // this is now pre-initialized for speed...
////        QR_sa[sa_index] = 0;
////        for (auto QCi_sa : QC_sa)
////        {
////          QCi_sa[sa_index] = 0;
////        }
//      }
//    }
//
//    return rollout(s);
//  }

  // select an action (with an exploration tradeoff)
  size_t a_id = greedyPolicy(s, exploration_constant, false);

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
  for (size_t i = 0; i < QC_sa.size(); i ++)
  {
    QC_sa[i][sa_index] += (total_costs[i + 1] - QC_sa[i][sa_index])/N_sa[sa_index];
  }

  return total_costs;
}

// TODO: parameter for selecting only a non-constraint-violating action (or uniform if that's impossible)
size_t MCTSRewardSolver::greedyPolicy(StateWithTime s, double kappa, bool constrained)
{
  size_t s_index = getIndexS(s);

  if (N_s[s_index] == 0)
  {
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
    while (!isValidAction(s.waypoint_id, a_id))
    {
      // if an invalid action was picked randomly, arbitrarily resolve to previous action (observe (id:0) always valid)
      a_id --;
    }

    return a_id;
  }

  vector<size_t> best_actions;
  vector<size_t> best_constrained_actions;
  double best_q = std::numeric_limits<double>::lowest();
  double best_q_constrained = std::numeric_limits<double>::lowest();
  for (size_t i = 0; i < actions.size(); i ++)
  {
    if (isValidAction(s.waypoint_id, i))
    {
      size_t sa_index = getIndexSA(s.waypoint_id, s.time_index, i);

      double q = QR_sa[sa_index];

      if (kappa != 0)
      {
        if (N_sa[sa_index] == 0)
        {
          q = kappa * 10000000;
        }
        else
        {
          q += kappa * sqrt(log(N_s[s_index]) / N_sa[sa_index]);
        }
      }

      if (q > best_q)
      {
        bool violates_constraints = false;
        for (size_t j = 0; j < QC_sa.size(); j++)
        {
          if (QC_sa[j][sa_index] > cost_constraints[j])
          {
            violates_constraints = true;
            break;
          }
        }

        if (violates_constraints)
        {
          if (q > best_q_constrained)
          {
            best_constrained_actions.clear();
            best_constrained_actions.push_back(i);
            best_q_constrained = q;
          }
          else if (q == best_q_constrained)
          {
            best_constrained_actions.push_back(i);
          }
        }
        else
        {
          best_actions.clear();
          best_actions.push_back(i);
          best_q = q;
        }
      }
      else if (q == best_q)
      {
        bool violates_constraints = false;
        for (size_t j = 0; j < QC_sa.size(); j++)
        {
          if (QC_sa[j][sa_index] > cost_constraints[j])
          {
            violates_constraints = true;
            break;
          }
        }

        if (violates_constraints)
        {
          if (q > best_q_constrained)
          {
            best_constrained_actions.clear();
            best_constrained_actions.push_back(i);
            best_q_constrained = q;
          }
          else if (q == best_q_constrained)
          {
            best_constrained_actions.push_back(i);
          }
        }
        else
        {
          best_actions.push_back(i);
        }
      }
    }
  }

  // currently select uniformly as an approximation to avoid solving a convex optimization problem
  size_t best_action;
  if (constrained)
  {
    if (!best_actions.empty())
    {
      best_action = uniformSelect(best_actions);
    }
    else
    {
      best_action = 0;
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
            best_action = i;
            break;
          }
        }
      }
      while (!isValidAction(s.waypoint_id, best_action))
      {
        // if an invalid action was picked randomly, arbitrarily resolve to previous action (observe (id:0) always valid)
        best_action --;
      }
    }
  }
  else
  {
    if (best_constrained_actions.empty())
    {
      best_action = uniformSelect(best_actions);
    }
    else if (best_actions.empty())
    {
      best_action = uniformSelect(best_constrained_actions);
    }
    else
    {
      if (static_cast<double>(rand()) / RAND_MAX > 0.5)
      {
        best_action = uniformSelect(best_actions);
      }
      else
      {
        best_action = uniformSelect(best_constrained_actions);
      }
    }
  }

  return best_action;
}

size_t MCTSRewardSolver::uniformSelect(vector<size_t> actions)
{
  size_t best_action = actions[0];
  if (actions.size() > 1)
  {
    double p = 0;
    double increment = 1.0/actions.size();
    double selection = static_cast<double>(rand())/RAND_MAX;
    for (auto a : actions)
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

void MCTSRewardSolver::setConstraints(vector<double> constraints)
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

StateWithTime MCTSRewardSolver::simulate_action(StateWithTime s, Action a, vector<double> &result_costs)
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

size_t MCTSRewardSolver::getIndexSA(size_t waypoint_id, size_t t, size_t action_id)
{
  return index_map[waypoint_id][t][action_id];
}

size_t MCTSRewardSolver::getIndexSA(size_t state_id, size_t action_id)
{
  return getIndexSA(states[state_id].waypoint_id, states[state_id].time_index, action_id);
}

size_t MCTSRewardSolver::getIndexS(StateWithTime s)
{
  return getIndexS(s.waypoint_id, s.time_index);
}

size_t MCTSRewardSolver::getIndexS(size_t waypoint_id, size_t t)
{
  return waypoint_id*(t_end + 1) + t;
}

void MCTSRewardSolver::loadTrajectory(std::string file_name)
{
  string trajectory_file_path = ros::package::getPath("waypoint_planner") + "/config/" + file_name;

  trajectory = EnvironmentSetup::readHumanTrajectory(trajectory_file_path);
}

void MCTSRewardSolver::loadWaypoints(string file_name)
{
  string waypoint_file_path = ros::package::getPath("waypoint_planner") + "/config/" + file_name;
  EnvironmentSetup::readWaypoints(waypoint_file_path, waypoints);
}

Action MCTSRewardSolver::getAction(geometry_msgs::Point s, double t)
{
  return getAction(s, static_cast<size_t>(t / time_step));
}

Action MCTSRewardSolver::getAction(geometry_msgs::Point s, size_t t)
{
  return actions[greedyPolicy(StateWithTime(waypointToIndex(s), t), 0, true)];
}

size_t MCTSRewardSolver::waypointToIndex(geometry_msgs::Point w)
{
//  for (size_t i = 0; i < waypoints.size(); i ++)
//  {
//    if (waypoints[i].x == w.x && waypoints[i].y == w.y && waypoints[i].z == w.z)
//    {
//      return i;
//    }
//  }
  size_t h = waypointHash(w);
  if (waypoint_index_map.count(h) == 1)
  {
    return waypoint_index_map[h];
  }
  return waypoints.size();  // error case, waypoint not found in list
}

size_t MCTSRewardSolver::waypointHash(geometry_msgs::Point w)
{
  return hasher({w.x, w.y, w.z});
}

double MCTSRewardSolver::reward(size_t state_id, size_t action_id, uint8_t mode)
{
  return SMDPFunctions::reward(State(waypoints[states[state_id].waypoint_id],
                                     trajectory.getPose(states[state_id].time_index*time_step)),
                               actions[action_id], mode);
}

bool MCTSRewardSolver::isValidAction(size_t waypoint_id, size_t action_id)
{
  return !(actions[action_id].actionType() == Action::MOVE
    && waypoints[waypoint_id].x == actions[action_id].actionGoal().x
    && waypoints[waypoint_id].y == actions[action_id].actionGoal().y
    && waypoints[waypoint_id].z == actions[action_id].actionGoal().z);
}
