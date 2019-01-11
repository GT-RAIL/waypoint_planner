#include "waypoint_planner/LPSolver.h"

using std::string;
using std::vector;
using std::cout;
using std::endl;

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

void LPSolver::constructModel(vector<double> total_costs)
{
  this->total_costs.clear();
  for (unsigned int i = 0; i < total_costs.size(); i ++)
  {
    this->total_costs.push_back(total_costs[i]);
  }

  cout << "Constructing LP model..." << endl;

  // construct lp with |S_t|*|A| variables
  int num_variables = states.size() * actions.size();
  lp = make_lp(0, num_variables);
  cout << "Model created with " << num_variables << " variables." << endl;

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
  cout << "Objective function added to model." << endl;

  // define the constraints row-by-row
  set_add_rowmode(lp, TRUE);

  // state occupancy constraints
  cout << "Calculating state occupancy constraints for " << states.size() << " states..." << endl;
  int updated = 0;
  for (size_t i = 0; i < states.size(); i ++)
  {
    if (i < 5 || i % 100 == 0)
      cout << "\t" << (static_cast<double>(i)/(states.size() - 1))*100 << "%, " << updated << " transition probabilities used..." << endl;

    // constraint for each s'
    REAL crow[1 + num_variables] = {0};

    // left side (iterate over actions)
    for (size_t j = 0; j < actions.size(); j ++)
    {
      crow[i*actions.size() + j + 1] = 1;
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
        vector<geometry_msgs::Point> s_primes;
        vector<double> transition_ps;
        SMDPFunctions::transitionModel(waypoints[states[j].waypoint_id], actions[k], s_primes, transition_ps);

        // iterate through possible waypoint transitions to calculate T(s'|s,a) (without time)
        for (size_t l = 0; l < s_primes.size(); l++)
        {
          if (states[i].waypoint_id != waypointToIndex(s_primes[l]))
            continue;

          size_t t_waypoint_id = waypointToIndex(s_primes[l]);
          double t_waypoint_p = transition_ps[l];

          vector<double> dts;
          vector<double> dt_ps;
          actions[k].duration(waypoints[states[j].waypoint_id], s_primes[j], dts, dt_ps);
          // iterate through possible time durations to fully calculate T(s'|s,a) (with time)
          for (size_t m = 0; m < dts.size(); m ++)
          {
            size_t t_time_id = states[j].time_index + static_cast<size_t>(ceil(dts[m]/time_step));
            if (states[i].time_index != t_time_id)
              continue;

            double t_prob = t_waypoint_p*dt_ps[m];
            if (t_time_id <= t_end && t_prob > 0)
            {
              crow[j*actions.size() + k + 1] -= t_prob;
              updated ++;
            }
          }
        }
      }
    }

    // define constant based on initial condition and add constraint
    // TODO: for now, try only first waypoint as initial state... maybe try all t=0 as initial states in the future
    double b = 0;
    if (i == 0)
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

  set_add_rowmode(lp, FALSE);

  // maximize objective function
  set_maxim(lp);

  // note: all variables must be >= 0 by default, so this constraint doesn't have to be added
  cout << "LP model constructed successfully!" << endl;

//  free(row);
}

void LPSolver::solveModel(double timeout)
{
  int suboptimal_solutions = 0;
  int total_attempts = 0;

  bool finished = false;

  while (!finished)
  {
    set_timeout(lp, timeout);

    total_attempts ++;

    cout << "Simplifying model for linearly dependent rows..." << endl;
    set_presolve(lp, PRESOLVE_ROWS | PRESOLVE_LINDEP, get_presolveloops(lp));
    cout << "Simplification complete." << endl;

    cout << "Attempting to solve model..." << endl;
    int success = solve(lp);

    cout << "Model finished, with code: " << success << endl;
    if (success == 1)
      suboptimal_solutions ++;
    cout << "Current progress: " << suboptimal_solutions << "/" << total_attempts << endl;
    cout << endl;
    if (success == 0)
      break;

    free(lp);

    constructModel(this->total_costs);
  }

  cout << "Objective value: " << get_objective(lp) << endl;

  REAL constr_results[get_Nrows(lp)];
  get_constraints(lp, constr_results);
  cout << "Constraint 1: " << constr_results[get_Nrows(lp) - 2] << endl;
  cout << "Constraint 2: " << constr_results[get_Nrows(lp) - 1] << endl;

  cout << "Retrieving results..." << endl;
  REAL vars[states.size() * actions.size()];
  get_variables(lp, vars);
  cout << "Copying results to this object for future use..." << endl;
  for (int i = 0; i < states.size() * actions.size(); i++)
  {
    ys.push_back(vars[i]);
  }
  cout << "Writing results to file in current directory..." << endl;
  std::ofstream var_file(ros::package::getPath("waypoint_planner") + "/config/var_results.txt");
  if (var_file.is_open())
  {
    for (int i = 0; i < states.size() * actions.size(); i++)
    {
      var_file << vars[i] << endl;
    }
  }
  else
  {
    cout << "Could not open file.  Freeing up memory and returning." << endl;
    return;
  }

  cout << "Results written.  Freeing up memory and returning." << endl;
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
  }
}

void LPSolver::costConstraint(uint8_t mode, double threshold)
{
  REAL crow[1 + states.size() * actions.size()] = {0};
  for (size_t i = 0; i < states.size(); i ++)
  {
    for (size_t j = 0; j < actions.size(); j ++)
    {
      crow[i*actions.size() + j + 1] = reward(i, j, mode);
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
