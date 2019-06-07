#include "waypoint_planner/LPSolver.h"

using std::string;
using std::vector;
using std::cout;
using std::endl;

LPSolver::LPSolver(double horizon, double step, string trajectory_file_name, string waypoint_file_name)
{
  t0 = 0;
  time_horizon = horizon;
  time_step = step;
  t_end = static_cast<size_t>(time_horizon / time_step);

  output_file_modifier = "results";

  cout << "Setting time scale to end at time index: " << t_end << endl;

  loadTrajectory(move(trajectory_file_name));
  loadWaypoints(move(waypoint_file_name));
  cout << "Loaded " << waypoints.size() << " waypoints." << endl;

  // initialize list of states
  for (auto waypoint : waypoints)
  {
    perch_states.emplace_back(PerchState(waypoint, false));
    perch_states.emplace_back(PerchState(waypoint, true));
  }

  for (size_t i = 0; i < perch_states.size(); i ++)
  {
    for (size_t j = 0; j <= t_end; j ++)
    {
      states.emplace_back(StateWithTime(i, j));
    }
  }

  cout << "Initialized " << states.size() << " states." << endl;

  SMDPFunctions::initializeActions(waypoints, actions);

  cout << "Initialized " << actions.size() << " actions." << endl;

  cout << "Constructing (s(t), a) list..." << endl;

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
}

void LPSolver::reset(double horizon, std::string trajectory_file_name, string output_file_modifier)
{
  t0 = 0;
  time_horizon = horizon;
  t_end = static_cast<size_t>(time_horizon / time_step);

  this->output_file_modifier = output_file_modifier;

  cout << "Setting time scale to end at time index: " << t_end << endl;

  loadTrajectory(move(trajectory_file_name));

  // initialize list of states

  states.clear();
  for (size_t i = 0; i < perch_states.size(); i ++)
  {
    for (size_t j = 0; j <= t_end; j ++)
    {
      states.emplace_back(StateWithTime(i, j));
    }
  }

  cout << "Initialized " << states.size() << " states." << endl;

  cout << "Constructing (s(t), a) list..." << endl;

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
          num_variables ++;
        }
        else
        {
          index_map[i][j][k] = std::numeric_limits<size_t>::max();
        }
      }
    }
  }
}

void LPSolver::resolve(double horizon, double step, size_t t0)
{
  time_horizon = horizon;
  this->t0 = t0;
  t_end = t0 + static_cast<size_t>(time_horizon / time_step);

  cout << "Setting time scale to end at time index: " << t_end << endl;

  // initialize list of states
  states.clear();
  for (size_t i = 0; i < perch_states.size(); i ++)
  {
    for (size_t j = t0; j <= t_end; j ++)
    {
      states.emplace_back(StateWithTime(i, j));
    }
  }

  cout << "Initialized " << states.size() << " states." << endl;

  cout << "Constructing (s(t), a) list..." << endl;

  num_variables = 0;
  index_map.resize(perch_states.size());
  for (size_t i = 0; i < perch_states.size(); i ++)
  {
    index_map[i].resize((t_end - t0) + 1);
    for (size_t j = t0; j <= t_end; j ++)
    {
      index_map[i][j-t0].resize(actions.size());
      for (size_t k = 0; k < actions.size(); k ++)
      {
        if (isValidAction(i, k))
        {
          index_map[i][j-t0][k] = num_variables;
          num_variables ++;
        }
        else
        {
          index_map[i][j-t0][k] = std::numeric_limits<size_t>::max();
        }
      }
    }
  }
}

void LPSolver::constructModel(vector<double> total_costs)
{
  constructModel(move(total_costs), PerchState(waypoints[0], false));
}

void LPSolver::constructModel(vector<double> total_costs, PerchState s0)
{
  this->total_costs.clear();
  for (unsigned int i = 0; i < total_costs.size(); i ++)
  {
    this->total_costs.push_back(total_costs[i]);
  }

  cout << "Constructing LP model..." << endl;

  // construct lp with |S_t|*|A| variables (for A valid depending on the state)
  lp = make_lp(0, num_variables);
  cout << "Model created with " << num_variables << " variables." << endl;

  // define objective function
  REAL row[1 + num_variables];
  for (size_t i = 0; i < states.size(); i++)
  {
    for (size_t j = 0; j < actions.size(); j ++)
    {
      // note: lp_solve API uses 1-indexing, so all row indices have +1 at the end
      row[getIndex(i, j) + 1] = reward(i, j, SMDPFunctions::REWARD);
    }
  }
  set_obj_fn(lp, row);
  cout << "Objective function added to model." << endl;

  // define the constraints row-by-row
  set_add_rowmode(lp, TRUE);

  // state occupancy constraints
  cout << "Calculating state occupancy constraints for " << states.size() << " states..." << endl;
  int updated = 0;
  size_t s0_i = perchStateToIndex(s0);
  for (size_t i = 0; i < states.size(); i ++)
  {
    if (i < 5 || i % 100 == 0)
      cout << "\t" << (static_cast<double>(i)/(states.size() - 1))*100 << "%, " << updated << " transition probabilities used..." << endl;

    // constraint for each s'
    REAL crow[1 + num_variables];
    for (size_t j = 0; j < 1 + num_variables; j ++)
    {
      crow[j] = 0;
    }

    // left side (iterate over actions)
    for (size_t j = 0; j < actions.size(); j ++)
    {
      if (isValidAction(states[i].state_id, j))
      {
        crow[getIndex(i, j) + 1] = 1;
      }
    }

    // right side (iterate over states and actions)
    for (size_t j = 0; j < states.size(); j ++)
    {
      // for speedup: states with greater than or equal to time indices cannot transition to the current state
      if (states[j].time_index >= states[i].time_index)
      {
        continue;
      }

      for (size_t k = 0; k < actions.size(); k ++)
      {
        if (!isValidAction(states[j].state_id, k))
        {
          continue;
        }

        vector<PerchState> s_primes;
        vector<double> transition_ps;
        SMDPFunctions::transitionModel(perch_states[states[j].state_id], actions[k], s_primes, transition_ps);

        // iterate through possible waypoint transitions to calculate T(s'|s,a) (without time)
        for (size_t l = 0; l < s_primes.size(); l++)
        {
          if (states[i].state_id != perchStateToIndex(s_primes[l]))
            continue;

          size_t t_state_id = perchStateToIndex(s_primes[l]);
          double t_state_p = transition_ps[l];

          vector<double> dts;
          vector<double> dt_ps;
          actions[k].duration(perch_states[states[j].state_id].waypoint, s_primes[l].waypoint, dts, dt_ps);
          // iterate through possible time durations to fully calculate T(s'|s,a) (with time)
          for (size_t m = 0; m < dts.size(); m ++)
          {
            size_t t_time_id = states[j].time_index + static_cast<size_t>(ceil(dts[m]/time_step));
            if (states[i].time_index != t_time_id)
              continue;

            double t_prob = t_state_p*dt_ps[m];
            if (t_time_id <= t_end && t_prob > 0)
            {
              crow[getIndex(j, k) + 1] -= t_prob;
              updated ++;
            }
          }
        }
      }
    }

    // define constant based on initial condition and add constraint
    // TODO: for now, try only given perch state as initial state...
    double b = 0;
    if (i == s0_i)
    {
      b = 1;
    }
    add_constraint(lp, crow, EQ, b);
  }
  cout << "State occupancy constraints added." << endl;

  // total cost constraints
  costConstraint(SMDPFunctions::COLLISION, this->total_costs[0]);
  cout << "Collision constraint added." << endl;
  costConstraint(SMDPFunctions::INTRUSION, this->total_costs[1]);
  cout << "Intrusion constraint added." << endl;
  costConstraint(SMDPFunctions::POWER, this->total_costs[2]);
  cout << "Power consumption constraint added." << endl;

  set_add_rowmode(lp, FALSE);

  // maximize objective function
  set_maxim(lp);

  // solver settings
//  set_scaling(lp, SCALE_MEAN | SCALE_LOGARITHMIC | SCALE_INTEGERS);
  set_scaling(lp, SCALE_MEAN | SCALE_INTEGERS);

  // note: all variables must be >= 0 by default, so this constraint doesn't have to be added
  cout << "LP model constructed successfully!" << endl;

//  free(row);
}

void LPSolver::setScaling(int scaling_type)
{
  if (scaling_type == 0)
  {
//    set_scaling(lp, SCALE_MEAN | SCALE_LOGARITHMIC | SCALE_INTEGERS);
    set_scaling(lp, SCALE_MEAN | SCALE_INTEGERS);
  }
  else if (scaling_type == 1)
  {
//    set_scaling(lp, SCALE_GEOMETRIC | SCALE_INTEGERS);
    set_scaling(lp, SCALE_GEOMETRIC | SCALE_INTEGERS);
  }
  else
  {
//    set_scaling(lp, SCALE_CURTISREID | SCALE_INTEGERS);
    set_scaling(lp, SCALE_RANGE | SCALE_LOGARITHMIC | SCALE_INTEGERS);
  }
}

bool LPSolver::solveModel(double timeout)
{
  int suboptimal_solutions = 0;
  int total_attempts = 0;

  set_timeout(lp, timeout);

  total_attempts ++;

  cout << "Simplifying model for linearly dependent rows..." << endl;
//  set_presolve(lp, PRESOLVE_ROWS | PRESOLVE_LINDEP, get_presolveloops(lp));
  cout << "Simplification complete." << endl;

  cout << "Attempting to solve model..." << endl;
  int success = solve(lp);

  cout << "Model finished, with code: " << success << endl;
  if (success != 0)
  {
    return false;
  }


  cout << "Objective value: " << get_objective(lp) << endl;

  REAL constr_results[get_Nrows(lp)];
  get_constraints(lp, constr_results);
  cout << "Constraint 1: " << constr_results[get_Nrows(lp) - 3] << endl;
  cout << "Constraint 2: " << constr_results[get_Nrows(lp) - 2] << endl;
  cout << "Constraint 3: " << constr_results[get_Nrows(lp) - 1] << endl;

  cout << "Retrieving results..." << endl;
  REAL vars[num_variables];
  get_variables(lp, vars);
  cout << "Copying results to this object for future use..." << endl;
  ys.clear();
  for (int i = 0; i < num_variables; i++)
  {
    ys.push_back(vars[i]);
  }
  cout << "Writing results to file in current directory..." << endl;
  std::ofstream var_file(ros::package::getPath("waypoint_planner") + "/config/var_" + output_file_modifier + ".txt");
  if (var_file.is_open())
  {
    for (int i = 0; i < num_variables; i++)
    {
      var_file << vars[i] << endl;
    }
  }
  else
  {
    cout << "Could not open file.  Freeing up memory and returning." << endl;
    freeModel();
    return false;
  }

  cout << "Results written.  Freeing up memory and returning." << endl;
  freeModel();

//  cout << "***********************************************\nnonzero values: " << endl;
//  for (size_t n = 0; n < perch_states.size(); n ++)
//  {
//    for (size_t i = 0; i < t_end + 1; i++)
//    {
//      for (size_t j = 0; j < actions.size(); j++)
//      {
//        if (isValidAction(n, j))
//        {
//          if (ys[getIndex(n, i, j)] > 0.00001)
//            cout << "\tw" << waypointToIndex(perch_states[n].waypoint) << "-" << perch_states[n].perched
//               << "(" << i << "), a" << j << ": " << ys[getIndex(n, i, j)] << endl;
//        }
//      }
//    }
//  }
//  cout << "***********************************************" << endl;

  return true;
}

void LPSolver::freeModel()
{
  free(lp);
}

void LPSolver::loadModel(string file_name)
{
  ys.clear();
  string model_file_path = ros::package::getPath("waypoint_planner") + "/config/" + file_name;
  std::ifstream model_file(model_file_path);
  string line;
  if (model_file.is_open())
  {
    while (getline(model_file, line))
    {
      ys.push_back(atof(line.c_str()));
    }
    model_file.close();
//    cout << "***********************************************\nnonzero values: " << endl;
//    for (size_t n = 0; n < perch_states.size(); n ++)
//    {
//      for (size_t i = 0; i < t_end + 1; i++)
//      {
//        for (size_t j = 0; j < actions.size(); j++)
//        {
//          if (isValidAction(n, j))
//          {
//            if (ys[getIndex(n, i, j)] > 0.00001)
//              cout << "\tw" << waypointToIndex(perch_states[n].waypoint) << "-" << perch_states[n].perched
//                << "(" << i << "), a" << j << ": " << ys[getIndex(n, i, j)] << endl;
//          }
//        }
//      }
//    }
//    cout << "***********************************************" << endl;
    cout << "LP solution loaded." << endl;
  }
}

void LPSolver::costConstraint(uint8_t mode, double threshold)
{
  REAL crow[num_variables + 1];
  for (unsigned int i = 0; i < num_variables + 1; i ++)
  {
    crow[i] = 0;
  }

  for (size_t i = 0; i < states.size(); i ++)
  {
    for (size_t j = 0; j < actions.size(); j ++)
    {
      if (isValidAction(states[i].state_id, j))
      {
        crow[getIndex(i, j) + 1] = reward(i, j, mode);
      }
    }
  }
  add_constraint(lp, crow, LE, threshold);

//  free(crow);
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

Action LPSolver::getAction(PerchState s, double t)
{
  return getAction(s, static_cast<size_t>(t / time_step));
}

Action LPSolver::getAction(PerchState s, size_t t)
{
  size_t state_id = perchStateToIndex(s);
  cout << "Getting action for time step " << t << ", waypoint " << waypointToIndex(s.waypoint)
       << " (perched: " << s.perched << ")..." << endl;
  size_t best_action_id = 0;
  double max_value = 0;
  double sum = 0;
  vector<size_t> possible_actions;
  vector<double> ps;
  for (size_t i = 0; i < actions.size(); i ++)
  {
    if (isValidAction(state_id, i))
    {
      double test_value = ys[getIndex(state_id, t, i)];
      if (test_value > 0)
      {
        sum += test_value;
        possible_actions.push_back(i);
        ps.push_back(test_value);
      }
      // Old code: deterministic action selection
//      if (test_value > max_value)
//      {
//        max_value = test_value;
//        best_action_id = i;
//      }
    }
  }

  // normalize probabilities
  for (unsigned int i = 0; i < ps.size(); i ++)
  {
    ps[i] /= sum;
  }

  double n = static_cast<double>(rand())/RAND_MAX;
  double selected_p = 0;
  sum = 0;
  for (unsigned int i = 0; i < ps.size(); i ++)
  {
    sum += ps[i];
    if (sum > n)
    {
      best_action_id = possible_actions[i];
      selected_p = ps[i];
      break;
    }
  }

  string str;
  string mod;
  if (actions[best_action_id].actionType() == Action::OBSERVE)
    str = "Observe";
  else if (actions[best_action_id].actionType() == Action::PERCH)
  {
    str = "Perch";
  }
  else if (actions[best_action_id].actionType() == Action::PERCH)
  {
    str = "Unperch";
  }
  else
  {
    str = "Move";
    std::stringstream ss("");
    ss << waypointToIndex(actions[best_action_id].actionGoal());
    mod = ss.str();
  }

  cout << "Best action: " << str << mod << ", with probability " << selected_p << endl;
//  cout << "Best action: " << str << mod << endl;

  return actions[best_action_id];
}

size_t LPSolver::getIndex(size_t perch_state_id, size_t t, size_t action_id)
{
  return index_map[perch_state_id][t - t0][action_id];
}

size_t LPSolver::getIndex(size_t state_id, size_t action_id)
{
  return getIndex(states[state_id].state_id, states[state_id].time_index, action_id);
}

// TODO: better lookup (hash waypoints to indices maybe?)
size_t LPSolver::perchStateToIndex(PerchState s)
{
  for (size_t i = 0; i < perch_states.size(); i ++)
  {
    if (perch_states[i].perched == s.perched && perch_states[i].waypoint.x == s.waypoint.x
        && perch_states[i].waypoint.y == s.waypoint.y && perch_states[i].waypoint.z == s.waypoint.z)
    {
      return i;
    }
  }
  return perch_states.size();  // error case, waypoint not found in list
}

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
  return SMDPFunctions::reward(State(perch_states[states[state_id].state_id].waypoint,
                                     perch_states[states[state_id].state_id].perched,
                                     trajectory.getPose(states[state_id].time_index*time_step)),
                               actions[action_id], mode);
}

bool LPSolver::isValidAction(size_t state_id, size_t action_id)
{
  return SMDPFunctions::isValidAction(perch_states[state_id], actions[action_id]);
}