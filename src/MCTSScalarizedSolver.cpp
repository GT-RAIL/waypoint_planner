#include "waypoint_planner/MCTSScalarizedSolver.h"

using std::string;
using std::vector;
using std::cout;
using std::endl;
using std::max;
using std::min;

const uint64_t MCTSScalarizedSolver::XORSHIFT_MAX = std::numeric_limits<uint64_t>::max();

MCTSScalarizedSolver::MCTSScalarizedSolver(double horizon, double step, string trajectory_file_name, string waypoint_file_name,
    vector<double> weights, double timeout_sec, size_t max_time_step_search_depth, double exploration_constant,
    int num_threads)
//  generator(std::chrono::system_clock::now().time_since_epoch().count()),
//  uniform_dist(0.0, 1.0),
//  bernoulli_dist(0.5)
{
  this->num_threads = num_threads;
  xorshift_state = static_cast<uint64_t>(std::chrono::system_clock::now().time_since_epoch().count());

  time_horizon = horizon;
  time_step = step;
  t_end = static_cast<size_t>(time_horizon / time_step);

  this->weights.resize(weights.size());
  for (size_t i = 0; i < weights.size(); i ++)
  {
    this->weights[i] = weights[i];
  }

  cout << "Setting time scale to end at time index: " << t_end << endl;

  loadTrajectory(move(trajectory_file_name));
  loadWaypoints(move(waypoint_file_name));
  cout << "Loaded " << waypoints.size() << " waypoints." << endl;

  SMDPFunctions::initializeActions(waypoints, actions);
  cout << "Initialized " << actions.size() << " actions." << endl;

  // initialize list of states
  for (auto waypoint : waypoints)
  {
    perch_states.emplace_back(PerchState(waypoint, false));
    perch_states.emplace_back(PerchState(waypoint, true));
  }

  // initialize list of states, waypoint hashes
  for (size_t i = 0; i < perch_states.size(); i ++)
  {
    size_t s_hash = stateHash(perch_states[i]);
    state_index_map[s_hash] = i;
    for (size_t j = 0; j <= t_end; j++)
    {
      states.emplace_back(StateWithTime(i, j));
      N_s[getIndexS(i, j)] = 0;
    }
  }

  cout << "Initialized " << states.size() << " states, and initialized MCTS state-based containers." << endl;

  cout << "Constructing (s(t), a) index list and initializing MCTS state-action-based containers..." << endl;

  num_variables = 0;
  index_map.resize(perch_states.size());
  for (size_t i = 0; i < perch_states.size(); i ++)
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
          num_variables ++;
        }
        else
        {
          index_map[i][j][k] = std::numeric_limits<size_t>::max();
        }
      }
    }
  }

  cout << "Initialized MCTS state-action-based containers." << endl;

  default_human_dims.x = 0.5;
  default_human_dims.y = 0.4;
  default_human_dims.z = 1.4;

  timeout = ros::Duration(timeout_sec);
  max_search_time_step_dst = max_time_step_search_depth;

  this->exploration_constant = exploration_constant;

  num_costs = 0;  // costs are included in the reward using scalarization
}

void MCTSScalarizedSolver::reset(double horizon, std::string trajectory_file_name)
{
  time_horizon = horizon;
  t_end = static_cast<size_t>(time_horizon / time_step);

  cout << "Setting time scale to end at time index: " << t_end << endl;

  loadTrajectory(move(trajectory_file_name));

  states.clear();
  // initialize list of states, waypoint hashes
  for (size_t i = 0; i < perch_states.size(); i ++)
  {
    size_t s_hash = stateHash(perch_states[i]);
    state_index_map[s_hash] = i;
    for (size_t j = 0; j <= t_end; j++)
    {
      states.emplace_back(StateWithTime(i, j));
      N_s[getIndexS(i, j)] = 0;
    }
  }

  cout << "Initialized " << states.size() << " states, and initialized MCTS state-based containers." << endl;

  cout << "Constructing (s(t), a) index list and initializing MCTS state-action-based containers..." << endl;

  num_variables = 0;
  index_map.resize(perch_states.size());
  for (size_t i = 0; i < perch_states.size(); i ++)
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
          num_variables ++;
        }
        else
        {
          index_map[i][j][k] = std::numeric_limits<size_t>::max();
        }
      }
    }
  }

  cout << "Initialized MCTS state-action-based containers." << endl;
}

uint64_t MCTSScalarizedSolver::xorshift64()
{
  rand_mutex.lock();
  uint64_t x = xorshift_state;
  x ^= x << 13;
  x ^= x >> 7;
  x ^= x << 17;
  xorshift_state = x;
  rand_mutex.unlock();
  return x;
}

Action MCTSScalarizedSolver::search(PerchState s, double t)
{
  return search(perchStateToIndex(s), static_cast<size_t>(t / time_step));
}

Action MCTSScalarizedSolver::search(size_t state_index, size_t time_step)
{
  return search(StateWithTime(state_index, time_step));
}

Action MCTSScalarizedSolver::search(StateWithTime s0)
{
  ros::Time start_time = ros::Time::now();
  ros::Time end_time = start_time + timeout;

  max_search_time_step = min(s0.time_index + max_search_time_step_dst, t_end);

  auto temp = boost::adaptors::values(N_s);
  std::fill(temp.begin(), temp.end(), 0);
  temp = boost::adaptors::values(N_sa);
  std::fill(temp.begin(), temp.end(), 0);
  temp = boost::adaptors::values(QR_sa);
  std::fill(temp.begin(), temp.end(), 0);

  size_t iterations = 0;
  while (ros::Time::now() < end_time)
  {
    vector<std::thread> threads;
    for (int i = 0; i < num_threads; i ++)
    {
      threads.emplace_back(std::thread(&MCTSScalarizedSolver::simulate, this, s0));
    }
    for (int i = 0; i < threads.size(); i ++)
    {
      threads[i].join();
      iterations ++;
    }
  }

  cout << "MCTS performed " << iterations << " rollouts over " << (ros::Time::now() - start_time).toSec() <<
    " seconds." << endl;

  size_t best_a_id = greedyPolicy(s0);

  cout << "Action selected: " << best_a_id << endl;
  cout << "\tExpected results: " << endl;
  for (size_t i = 0; i < actions.size(); i ++)
  {
    if (isValidAction(s0.state_id, i))
    {
      size_t sa_index = getIndexSA(s0.state_id, s0.time_index, i);
      cout << "\tAction " << i << ": QR(s,a)=" << QR_sa[sa_index] << "; N(s,a)=" << N_sa[sa_index] << endl;
    }
  }

  return actions[best_a_id];
}

vector<double> MCTSScalarizedSolver::simulate(StateWithTime s)
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

  size_t a_id;
  a_id = selectAction(s, exploration_constant);

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

  size_t sa_index = getIndexSA(s.state_id, s.time_index, a_id);

  lookup_mutex.lock();
  N_s[s_index] += 1;
  N_sa[sa_index] += 1;
  QR_sa[sa_index] += (total_costs[0] - QR_sa[sa_index]) / N_sa[sa_index];
  lookup_mutex.unlock();

  return total_costs;
}

size_t MCTSScalarizedSolver::greedyPolicy(StateWithTime s)
{
  size_t s_index = getIndexS(s);

  lookup_mutex.lock();
  if (N_s[s_index] == 0)
  {
    lookup_mutex.unlock();
    // select an action (with a uniform(?) policy)
    size_t a_id = actions.size() - 1;
//    if (bernoulli_dist(generator))  // take observe with probability 0.5, otherwise select uniformly
//    if (xorshift64() > 0.5*XORSHIFT_MAX)
//    {
    double p = 0.0;
    double increment = 1.0/actions.size();
//      double selection = uniform_dist(generator);
    double selection = static_cast<double>(xorshift64())/XORSHIFT_MAX;
    for (size_t i = 0; i < actions.size() - 1; i ++)
    {
      p += increment;
      if (p > selection)
      {
        a_id = i;
        break;
      }
    }
//      std::uniform_int_distribution<size_t> dist(0, actions.size() - 1);
//      a_id = dist(generator);
//    }
    while (!isValidAction(s.state_id, a_id))
    {
      // if an invalid action was picked randomly, arbitrarily resolve to previous action (observe (id:0) always valid)
      a_id --;
    }

    return a_id;
  }
  lookup_mutex.unlock();

  vector<size_t> best_actions;
  double best_q = std::numeric_limits<double>::lowest();
  for (size_t i = 0; i < actions.size(); i ++)
  {
    if (isValidAction(s.state_id, i))
    {
      size_t sa_index = getIndexSA(s.state_id, s.time_index, i);

      if (N_sa[sa_index] == 0)
      {
        continue;
      }

      double q = QR_sa[sa_index];

      if (q > best_q)
      {
        best_actions.resize(1);
        best_actions[0] = i;
        best_q = q;
      }
      else if (q == best_q)
      {
        best_actions.push_back(i);
      }
    }
  }

  if (best_actions.empty())
  {
    size_t a_id = actions.size() - 1;
    double p = 0.0;
    double increment = 1.0/actions.size();
    double selection = static_cast<double>(xorshift64())/XORSHIFT_MAX;
    for (size_t i = 0; i < actions.size() - 1; i ++)
    {
      p += increment;
      if (p > selection)
      {
        a_id = i;
        break;
      }
    }
    while (!isValidAction(s.state_id, a_id))
    {
      a_id --;
    }

    return a_id;
  }

  // currently select uniformly as an approximation to avoid solving a convex optimization problem
  return uniformSelect(best_actions);
}

size_t MCTSScalarizedSolver::selectAction(StateWithTime s, double kappa)
{
  size_t s_index = getIndexS(s);

  lookup_mutex.lock();
  if (N_s[s_index] == 0)
  {
    lookup_mutex.unlock();
    return 0;  // arbitrarily choose observe action, as any unexplored actions will be tried next
  }
  lookup_mutex.unlock();

  vector<size_t> best_actions;
  double best_q = std::numeric_limits<double>::lowest();
  for (size_t i = 0; i < actions.size(); i ++)
  {
    if (isValidAction(s.state_id, i))
    {
      size_t sa_index = getIndexSA(s.state_id, s.time_index, i);

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
        best_actions.resize(1);
        best_actions[0] = i;
        best_q = q;
      }
      else if (q == best_q)
      {
        best_actions.push_back(i);
      }
    }
  }

  // currently select uniformly as an approximation to avoid solving a convex optimization problem
  size_t best_action = uniformSelect(best_actions);

  return best_action;
}

size_t MCTSScalarizedSolver::uniformSelect(vector<size_t> &actions)
{
  size_t best_action = actions[0];
  if (actions.size() > 1)
  {
    double p = 0;
    double increment = 1.0/actions.size();
//    double selection = uniform_dist(generator);
    double selection = static_cast<double>(xorshift64())/XORSHIFT_MAX;
    for (auto a : actions)
    {
      p += increment;
      if (p > selection)
      {
        best_action = a;
        break;
      }
    }
//    std::uniform_int_distribution<size_t> dist(0, actions.size() - 1);
//    best_action = actions[dist(generator)];
  }

  return best_action;
}

void MCTSScalarizedSolver::setWeights(vector<double> weights)
{
  if (weights.size() != this->weights.size())
  {
    cout << "New weights do not match the number of costs!" << endl;
    cout << "Weights were not updated." << endl;
    return;
  }
  for (size_t i = 0; i < weights.size(); i ++)
  {
    this->weights[i] = weights[i];
  }
}

StateWithTime MCTSScalarizedSolver::simulate_action(StateWithTime s, Action a, vector<double> &result_costs)
{
  geometry_msgs::Point goal;
  if (a.actionType() == Action::MOVE)
  {
    goal = a.actionGoal();
  }
  else
  {
    goal = perch_states[s.state_id].waypoint;
  }

  vector<double> durations;
  vector<double> probabilities;
  a.duration(perch_states[s.state_id].waypoint, goal, durations, probabilities);
//  double n = uniform_dist(generator);
  double n = static_cast<double>(xorshift64())/XORSHIFT_MAX;
  double duration = durations[durations.size() - 1];
  double p = 0;
  for (size_t i = 0; i < probabilities.size() - 1; i ++)
  {
    p += probabilities[i];
    if (p >= n)
    {
      duration = durations[i];
      break;
    }
  }

  // calculate rewards and costs and add them to the totals
  if (a.actionType() == Action::OBSERVE)
  {
    result_costs[0] = (weights[0]*RewardsAndCosts::reward_recognition(trajectory.getPose(s.time_index * time_step),
                                                                     default_human_dims,
                                                                     perch_states[s.state_id].waypoint)
      + weights[1]*RewardsAndCosts::cost_collision(trajectory.getPose(s.time_index * time_step),
                                                   default_human_dims, perch_states[s.state_id].waypoint)
      + weights[2]*RewardsAndCosts::cost_intrusion(trajectory.getPose(s.time_index * time_step),
                                                   perch_states[s.state_id].waypoint, perch_states[s.state_id].perched)
      + weights[3]*RewardsAndCosts::cost_power(perch_states[s.state_id].perched, a)) * duration;
  }
  else if (a.actionType() == Action::PERCH || a.actionType() == Action::UNPERCH)
  {
    result_costs[0] = (weights[1]*RewardsAndCosts::cost_collision(trajectory.getPose(s.time_index * time_step),
                                                   default_human_dims, perch_states[s.state_id].waypoint)
      + weights[2]*RewardsAndCosts::cost_intrusion(trajectory.getPose(s.time_index * time_step),
                                                   perch_states[s.state_id].waypoint, perch_states[s.state_id].perched)
      + weights[3]*RewardsAndCosts::cost_power(perch_states[s.state_id].perched, a)) * duration;
  }
  else if (a.actionType() == Action::MOVE)
  {
    result_costs[0] = (weights[3]*RewardsAndCosts::cost_power(perch_states[s.state_id].perched, a)) * duration;
  }

  if (a.actionType() == Action::MOVE)
  {
    return StateWithTime(perchStateToIndex(PerchState(goal, false)),
        static_cast<size_t>(ceil(s.time_index + duration / time_step)));
  }
  else if (a.actionType() == Action::PERCH)
  {
    return StateWithTime(perchStateToIndex(PerchState(goal, true)),
        static_cast<size_t>(ceil(s.time_index + duration / time_step)));
  }
  else if (a.actionType() == Action::UNPERCH)
  {
    return StateWithTime(perchStateToIndex(PerchState(goal, false)),
                         static_cast<size_t>(ceil(s.time_index + duration / time_step)));
  }
  return StateWithTime(s.state_id, static_cast<size_t>(ceil(s.time_index + duration / time_step)));
}

size_t MCTSScalarizedSolver::getIndexSA(size_t perch_state_id, size_t t, size_t action_id)
{
  return index_map[perch_state_id][t][action_id];
}

size_t MCTSScalarizedSolver::getIndexSA(size_t state_id, size_t action_id)
{
  return getIndexSA(states[state_id].state_id, states[state_id].time_index, action_id);
}

size_t MCTSScalarizedSolver::getIndexS(StateWithTime s)
{
  return getIndexS(s.state_id, s.time_index);
}

size_t MCTSScalarizedSolver::getIndexS(size_t perch_state_id, size_t t)
{
  return perch_state_id*(t_end + 1) + t;
}

void MCTSScalarizedSolver::loadTrajectory(std::string file_name)
{
  string trajectory_file_path = ros::package::getPath("waypoint_planner") + "/config/" + file_name;

  trajectory = EnvironmentSetup::readHumanTrajectory(trajectory_file_path);
}

void MCTSScalarizedSolver::loadWaypoints(string file_name)
{
  string waypoint_file_path = ros::package::getPath("waypoint_planner") + "/config/" + file_name;
  EnvironmentSetup::readWaypoints(waypoint_file_path, waypoints);
}

Action MCTSScalarizedSolver::getAction(PerchState s, double t)
{
  return getAction(s, static_cast<size_t>(t / time_step));
}

Action MCTSScalarizedSolver::getAction(PerchState s, size_t t)
{
  return actions[greedyPolicy(StateWithTime(perchStateToIndex(s), t))];
}

size_t MCTSScalarizedSolver::perchStateToIndex(PerchState s)
{
  size_t h = stateHash(s);
  if (state_index_map.count(h) == 1)
  {
    return state_index_map[h];
  }
  return perch_states.size();  // error case, perch_state not found in list
}

size_t MCTSScalarizedSolver::stateHash(PerchState s)
{
  return hasher({s.waypoint.x, s.waypoint.y, s.waypoint.z, static_cast<double>(s.perched)});
}

//double MCTSRewardSolver::reward(size_t state_id, size_t action_id, uint8_t mode)
//{
//  return SMDPFunctions::reward(State(perch_states[states[state_id].state_id].waypoint,
//                                     perch_states[states[state_id].state_id].perched,
//                                     trajectory.getPose(states[state_id].time_index*time_step)),
//                               actions[action_id], mode);
//}

bool MCTSScalarizedSolver::isValidAction(size_t state_id, size_t action_id)
{
  return SMDPFunctions::isValidAction(perch_states[state_id], actions[action_id]);
}
