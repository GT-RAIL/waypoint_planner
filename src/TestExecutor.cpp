#include "waypoint_planner/TestExecutor.h"

using std::vector;
using std::cout;
using std::endl;
using std::string;

const uint8_t TestExecutor::SMDP = 0;
const uint8_t TestExecutor::LP_SOLVE = 1;
const uint8_t TestExecutor::LP_LOAD = 2;
const uint8_t TestExecutor::MCTS_CONSTRAINED = 3;
const uint8_t TestExecutor::MCTS_SCALARIZED = 4;

TestExecutor::TestExecutor(double horizon, double step, uint8_t approach, uint8_t mode, vector<double> weights,
    size_t search_depth, string trajectory_file, bool lp_resolve, double resolve_horizon) :
    solver(horizon, step, mode, trajectory_file, "iss_waypoints.csv", weights),    // TODO: parameters here for optional values
    lp_solver(horizon, step, trajectory_file, "iss_waypoints.csv"),    // TODO: parameters here for optional values
//    mcts_solver(horizon, step, "iss_trajectory.yaml", "iss_waypoints.csv", {1.0, 75.0}, 150.0,
//        static_cast<size_t>(horizon/step), 2.0),  // TODO: parameters here for optional values
    mcts_reward_solver(horizon, step, trajectory_file, "iss_waypoints.csv", weights, 30.0,
        search_depth, 2, 6),
    mcts_scalarized_solver(horizon, step, trajectory_file, "iss_waypoints_small.csv", weights, 10.0,
        search_depth, 100, 6),
    current_action(Action::OBSERVE),
    pnh("~")
{
  if (approach == TestExecutor::MCTS_CONSTRAINED)
  {
    c1_hat = weights[0];
    c2_hat = weights[1];
    c3_hat = weights[2];
    cout << "\tc1_hat: " << c1_hat << ", c2_hat: " << c2_hat << ", c3_hat: " << c3_hat << endl;
  }

  srand(time(NULL));

  this->approach = approach;
  this->mode = mode;
  this->search_depth = search_depth;

  this->weights.resize(weights.size());
  for (size_t i = 0; i < weights.size(); i ++)
  {
    this->weights[i] = weights[i];
  }

  string trajectory_file_path = ros::package::getPath("waypoint_planner") + "/config/" + trajectory_file;
  trajectory = EnvironmentSetup::readHumanTrajectory(trajectory_file_path);

  default_human_dims.x = 0.5;
  default_human_dims.y = 0.4;
  default_human_dims.z = 1.4;

  if (this->approach == LP_SOLVE)
  {
    //TODO: temp: commented out for multiple testing (will be set and solved with reset())
//    lp_solver.constructModel(weights);  // constraint thresholds {d1, d2, d3} packed into weights
//    lp_solver.solveModel(600);  // solver timeout (s) before restarting
//    ROS_INFO("LP model solved.");
  }
  else if (this->approach == LP_LOAD)
  {
    lp_solver.loadModel("var_results.txt");
    ROS_INFO("LP model loaded.");
  }
  else if (this->approach == SMDP)
  {
    solver.backwardsInduction();
    ROS_INFO("Policy computed.");
  }

  time_horizon = horizon;
  current_time = 0;
  next_decision = 0;
  time_step = step;
  search_depth_time = search_depth*time_step;
  this->resolve_horizon = resolve_horizon;
  this->lp_resolve = lp_resolve;

  if (this->approach == MCTS_CONSTRAINED)
  {
    //time-scaling for non-full-depth searches
    double time_scaling = search_depth_time / (time_horizon - current_time);
    if (time_scaling > 1.0)
    {
      time_scaling = 1.0;
    }

    mcts_reward_solver.setConstraints({time_scaling*c1_hat, time_scaling*c2_hat, time_scaling*c3_hat});
  }

  // TODO: better state initialization
  state.waypoint.x = 11.39;
  state.waypoint.y = -10.12;
  state.waypoint.z = 4.45;
  state.perched = false;

  r = 0;
  c1 = 0;
  c2 = 0;
  c3 = 0;

  robot_vis_publisher = pnh.advertise<visualization_msgs::Marker>("test_robot_vis", 1, this);
  human_sim_time_publisher = n.advertise<std_msgs::Float32>("human_simulator/time_update", 1, this);

  robot_marker.header.frame_id = "world";
  robot_marker.pose.position = state.waypoint;
  robot_marker.pose.orientation.w = 1.0;
  robot_marker.action = visualization_msgs::Marker::ADD;
  robot_marker.ns = "test_robot";
  robot_marker.id = 0;
  robot_marker.type = visualization_msgs::Marker::CUBE;
  robot_marker.scale.x = 0.25;
  robot_marker.scale.y = 0.25;
  robot_marker.scale.z = 0.25;
  robot_marker.color.r = 1.0;
  robot_marker.color.g = 0.0;
  robot_marker.color.b = 1.0;
  robot_marker.color.a = 1.0;
}

void TestExecutor::randomizeWeights()
{
  weights[0] = 4*static_cast<double>(rand())/RAND_MAX + 1;
  weights[1] = 50*static_cast<double>(rand())/RAND_MAX + 10;
  weights[2] = 50*static_cast<double>(rand())/RAND_MAX + 20;

  cout << "New weights: " << weights[0] << ", " << weights[1] << ", " << weights[2] << endl;
}

bool TestExecutor::reset(double horizon, string trajectory_file, string lp_model, bool solve, bool randomize_trajectory)
{
  if (approach == TestExecutor::SMDP)
  {
    solver.reset(horizon, trajectory_file);
  }
  else if (approach == TestExecutor::LP_SOLVE || approach == TestExecutor::LP_LOAD)
  {
    lp_solver.reset(horizon, trajectory_file, lp_model, randomize_trajectory);
  }
  else if (approach == TestExecutor::MCTS_CONSTRAINED)
  {
    mcts_reward_solver.reset(horizon, trajectory_file, weights);
  }
  else if (approach == TestExecutor::MCTS_SCALARIZED)
  {
    mcts_scalarized_solver.reset(horizon, trajectory_file);
  }

  if (approach == TestExecutor::MCTS_CONSTRAINED)
  {
    c1_hat = weights[0];
    c2_hat = weights[1];
    c3_hat = weights[2];
    cout << "\tc1_hat: " << c1_hat << ", c2_hat: " << c2_hat << ", c3_hat: " << c3_hat << endl;
  }

  if (randomize_trajectory)
  {
    // TODO: this is only implemented for lp_solver right now!
    if (approach == TestExecutor::LP_SOLVE)
    {
      trajectory = lp_solver.getTrajectory();
    }
    else
    {
      string trajectory_file_path = ros::package::getPath("waypoint_planner") + "/config/" + trajectory_file;
      trajectory = EnvironmentSetup::readHumanTrajectory(trajectory_file_path);
    }
  }
  else
  {
    string trajectory_file_path = ros::package::getPath("waypoint_planner") + "/config/" + trajectory_file;
    trajectory = EnvironmentSetup::readHumanTrajectory(trajectory_file_path);
  }

  // TODO: better state initialization
  state.waypoint.x = 11.39;
  state.waypoint.y = -10.12;
  state.waypoint.z = 4.45;
  state.perched = false;

  r = 0;
  c1 = 0;
  c2 = 0;
  c3 = 0;

  time_horizon = horizon;
  current_time = 0;
  next_decision = 0;
  search_depth_time = search_depth*time_step;

  robot_marker.header.frame_id = "world";
  robot_marker.pose.position = state.waypoint;
  robot_marker.pose.orientation.w = 1.0;
  robot_marker.action = visualization_msgs::Marker::ADD;
  robot_marker.ns = "test_robot";
  robot_marker.id = 0;
  robot_marker.type = visualization_msgs::Marker::CUBE;
  robot_marker.scale.x = 0.25;
  robot_marker.scale.y = 0.25;
  robot_marker.scale.z = 0.25;
  robot_marker.color.r = 1.0;
  robot_marker.color.g = 0.0;
  robot_marker.color.b = 1.0;
  robot_marker.color.a = 1.0;

  if (solve)
  {
    if (this->approach == LP_SOLVE)
    {
      lp_solver.constructModel(weights);  // constraint thresholds {d1, d2, d3} packed into weights
      if (!lp_solver.solveModel(1200))  // solver timeout (s) before restarting
      {
        return false;
      }
      ROS_INFO("LP model solved.");
    }
    else if (this->approach == LP_LOAD)
    {
      lp_solver.loadModel("var_" + lp_model + ".txt");
      ROS_INFO("LP model loaded.");
    }
    else if (this->approach == SMDP)
    {
      solver.backwardsInduction();
      ROS_INFO("Policy computed.");
    }

    if (this->approach == MCTS_CONSTRAINED)
    {
      //time-scaling for non-full-depth searches
      double time_scaling = search_depth_time / (time_horizon - current_time);
      if (time_scaling > 1.0)
      {
        time_scaling = 1.0;
      }

      mcts_reward_solver.setConstraints({time_scaling * c1_hat, time_scaling * c2_hat, time_scaling * c3_hat});
    }
  }

  return true;
}

bool TestExecutor::resolve()
{
  size_t t0 = static_cast<size_t>(current_time / time_step);
  if (current_time + resolve_horizon > time_horizon)
  {
    ROS_INFO("Within window, no need to resolve.");
    return true;
  }

  if (approach == TestExecutor::SMDP)
  {
//    solver.reset(horizon, trajectory_file);
  }
  else if (approach == TestExecutor::LP_SOLVE || approach == TestExecutor::LP_LOAD)
  {
    lp_solver.resolve(resolve_horizon, time_step, t0);
  }
  else if (approach == TestExecutor::MCTS_CONSTRAINED)
  {
//    mcts_reward_solver.reset(horizon, trajectory_file, weights);
  }
  else if (approach == TestExecutor::MCTS_SCALARIZED)
  {
//    mcts_scalarized_solver.reset(horizon, trajectory_file);
  }

//  if (approach == TestExecutor::MCTS_CONSTRAINED)
//  {
//    c1_hat = weights[0];
//    c2_hat = weights[1];
//    c3_hat = weights[2];
//    cout << "\tc1_hat: " << c1_hat << ", c2_hat: " << c2_hat << ", c3_hat: " << c3_hat << endl;
//  }

  //this needs to be time scaled based on resolve_horizon
  double time_scaling = resolve_horizon/(time_horizon - current_time);
  if (time_scaling > 1.0)
  {
    time_scaling = 1.0;
  }
  vector<double> w_hat;
  w_hat.push_back(time_scaling*(weights[0] - c1));
  w_hat.push_back(time_scaling*(weights[1] - c2));
  w_hat.push_back(time_scaling*(weights[2] - c3));

  // to still allow cases to be solved when budget gets used up due to stochastic policy
  if (w_hat[0] <= .01)
  {
    w_hat[0] = .01;
  }
  if (w_hat[1] <= .1)
  {
    w_hat[1] = .1;
  }
  if (w_hat[2] <= .125)
  {
    w_hat[2] = .125;
  }

  if (this->approach == LP_SOLVE)
  {
    lp_solver.constructModel(w_hat, state);  // constraint thresholds {d1, d2, d3} packed into weights
    if (!lp_solver.solveModel(1200))  // solver timeout (s) before restarting
    {
      return false;
    }
    ROS_INFO("LP model solved.");
  }
  else if (this->approach == LP_LOAD)
  {
//    lp_solver.loadModel("var_" + lp_model + ".txt");
//    ROS_INFO("LP model loaded.");
  }
  else if (this->approach == SMDP)
  {
//    solver.backwardsInduction();
//    ROS_INFO("Policy computed.");
  }

  if (this->approach == MCTS_CONSTRAINED)
  {
//    //time-scaling for non-full-depth searches
//    double time_scaling = search_depth_time / (time_horizon - current_time);
//    if (time_scaling > 1.0)
//    {
//      time_scaling = 1.0;
//    }
//
//    mcts_reward_solver.setConstraints({time_scaling*c1_hat, time_scaling*c2_hat, time_scaling*c3_hat});
  }

  return true;
}

bool TestExecutor::run(double sim_step, string log_name)
{
  if (current_time >= next_decision)
  {
    if (current_action.actionType() == Action::MOVE)
    {
      ROS_INFO("Move action complete.");
      state.waypoint = current_action.actionGoal();
      robot_marker.pose.position = state.waypoint;
      robot_marker.color.r = 1.0;
      robot_marker.color.g = 0.0;
      robot_marker.color.b = 1.0;
      robot_marker.color.a = 1.0;
    }
    else if (current_action.actionType() == Action::PERCH)
    {
      state.perched = true;
      robot_marker.color.r = 0.0;
      robot_marker.color.g = 1.0;
      robot_marker.color.b = 1.0;
      robot_marker.color.a = 1.0;
    }
    else if (current_action.actionType() == Action::UNPERCH)
    {
      state.perched = false;
      robot_marker.color.r = 1.0;
      robot_marker.color.g = 0.0;
      robot_marker.color.b = 1.0;
      robot_marker.color.a = 1.0;
    }

    if (lp_resolve)
    {
      if (!resolve())
      {
        ROS_INFO("Resolve failed!!!");
        return true;
      }
    }

    if (approach == SMDP)
    {
      current_action = solver.getAction(state, current_time);
    }
    else if (approach == LP_SOLVE || approach == LP_LOAD)
    {
      current_action = lp_solver.getAction(state, current_time);
    }
    else if (approach == MCTS_CONSTRAINED)
    {
      //current_action = mcts_solver.search(waypoint, current_time);
      current_action = mcts_reward_solver.search(state, current_time);
    }
    else if (approach == MCTS_SCALARIZED)
    {
      current_action = mcts_scalarized_solver.search(state, current_time);
    }

    geometry_msgs::Point goal;
    if (current_action.actionType() == Action::MOVE)
    {
      ROS_INFO("Starting move action.");
      goal = current_action.actionGoal();
      robot_marker.color.r = 1.0;
      robot_marker.color.g = 0.5;
      robot_marker.color.b = 0.0;
      robot_marker.color.a = 0.6;
    }
    else if (current_action.actionType() == Action::PERCH)
    {
      ROS_INFO("Starting perch action.");
      goal = state.waypoint;
      robot_marker.color.r = 0.0;
      robot_marker.color.g = 1.0;
      robot_marker.color.b = 1.0;
      robot_marker.color.a = 0.6;
    }
    else if (current_action.actionType() == Action::UNPERCH)
    {
      ROS_INFO("Starting unperch action.");
      goal = state.waypoint;
      robot_marker.color.r = 1.0;
      robot_marker.color.g = 0.0;
      robot_marker.color.b = 1.0;
      robot_marker.color.a = 0.6;
    }
    else
    {
      ROS_INFO("Observing.");
      goal = state.waypoint;
    }

    // determine a (fake) execution time
    vector<double> durations;
    vector<double> probabilities;
    current_action.duration(state.waypoint, goal, durations, probabilities);
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

    if (current_action.actionType() == Action::MOVE)
    {
      start_move_time = current_time;
      move_dur = duration;
      dx = goal.x - state.waypoint.x;
      dy = goal.y - state.waypoint.y;
      dz = goal.z - state.waypoint.z;
      startx = state.waypoint.x;
      starty = state.waypoint.y;
      startz = state.waypoint.z;
    }

    next_decision = current_time + duration;
    ROS_INFO("Action duration: %f", duration);

    double r0 = RewardsAndCosts::reward_recognition(trajectory.getPose(current_time),
        default_human_dims, state.waypoint)*duration;
    double c1_0 = RewardsAndCosts::cost_collision(trajectory.getPose(current_time), default_human_dims,
        state.waypoint)*duration;
    double c2_0 = RewardsAndCosts::cost_intrusion(trajectory.getPose(current_time), state.waypoint, state.perched)*duration;
    double c3_0 = RewardsAndCosts::cost_power(state.perched, current_action)*duration;
    if (approach == MCTS_CONSTRAINED)
    {
      // update cost thresholds
      c1_hat -= c1_0;
      c2_hat -= c2_0;
      c3_hat -= c3_0;
      if (c1_hat < 0.01)
      {
        c1_hat = 0.01;
      }
      if (c2_hat < 0.01)
      {
        c2_hat = 0.01;
      }
      if (c3_hat < 0.01)
      {
        c3_hat = 0.01;
      }

      //time-scaling for non-full-depth searches
      double time_scaling = search_depth_time / (time_horizon - current_time);
      if (time_scaling > 1.0)
      {
        time_scaling = 1.0;
      }

      //mcts_solver.setConstraints({c1_hat, c2_hat});
      mcts_reward_solver.setConstraints({time_scaling*c1_hat, time_scaling*c2_hat, time_scaling*c3_hat});
      //mcts_solver.updateConstraints(current_action.actionGoal(), next_decision);
    }

    // calculate rewards and costs and add them to the totals
    if (current_action.actionType() == Action::OBSERVE)
    {
      r += r0;
      c1 += c1_0;
      c2 += c2_0;
    }
    else if (current_action.actionType() == Action::PERCH || current_action.actionType() == Action::UNPERCH)
    {
      c1 += c1_0;
      c2 += c2_0;
    }
    c3 += c3_0;  // this accumulates for every action

    // log data, in the form:
    //  action type, goal x, goal y, goal z, c1 remaining, c2 remaining, c3 remaining, current x, current y, current z,
    //  perched, [remaining human trajectory: x, y, z, qx, qy, qz, qw, ...]
    trajectory.getPose(0);
    std::stringstream traj_stream;
    for (double t_remaining = current_time; t_remaining < time_horizon; t_remaining += time_step)
    {
      geometry_msgs::Pose traj_remaining_pose = trajectory.getPose(t_remaining);
      traj_stream << traj_remaining_pose.position.x << "," << traj_remaining_pose.position.y << ","
        << traj_remaining_pose.position.z << "," << traj_remaining_pose.orientation.x << ","
        << traj_remaining_pose.orientation.y << "," << traj_remaining_pose.orientation.z << ","
        << traj_remaining_pose.orientation.w << ",";
    }
    std::ofstream log_file;
    log_file.open(log_name, std::ios::out | std::ios::app);
    log_file << std::to_string(current_action.actionType()) << "," << current_action.actionGoal().x << ","
      << current_action.actionGoal().y << "," << current_action.actionGoal().z << "," << weights[0] - c1 << ","
      << weights[1] - c2 << "," << weights[2] - c3 << "," << state.waypoint.x << "," << state.waypoint.y << ","
      << state.waypoint.z << "," << state.perched << "," << traj_stream.str() << endl;
    log_file.close();

    std::cout << "Time: " << current_time << "\tReward: " << r << ", C1: " << c1 << ", C2: " << c2 << ", C3: " << c3
      << std::endl;
  }

  // update fake execution time
  current_time += sim_step;

  // update robot's position if moving
  if (current_action.actionType() == Action::MOVE)
  {
    double ratio = 1.0;
    if (current_time - start_move_time <= move_dur)
    {
      ratio = (current_time - start_move_time)/move_dur;
    }
    robot_marker.pose.position.x = startx + dx*ratio;
    robot_marker.pose.position.y = starty + dy*ratio;
    robot_marker.pose.position.z = startz + dz*ratio;
  }

  // send time update to human trajectory visualizer
  robot_vis_publisher.publish(robot_marker);
  std_msgs::Float32 change_time;
  change_time.data = current_time;
  human_sim_time_publisher.publish(change_time);

  return current_time > time_horizon;
}

void TestExecutor::reportResults()
{
  std::cout << "Trial complete." << std::endl;
  std::cout << "Total accumulated reward: " << r << std::endl;
  std::cout << "Total accumulated collision cost: " << c1 << std::endl;
  std::cout << "Total accumulated intrusion cost: " << c2 << std::endl;
  std::cout << "Total accumulated power cost: " << c3 << std::endl;
}

bool TestExecutor::retryLP(int scaling_type)
{
  lp_solver.setScaling(scaling_type);
  if (!lp_solver.solveModel(1200))  // solver timeout (s) before restarting
  {
    return false;
  }
  ROS_INFO("LP model solved.");
  return true;
}

void TestExecutor::freeLP()
{
  lp_solver.freeModel();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_executor");

  Approximator a;
  return EXIT_SUCCESS;

  vector<double> weights{1, 300, 300};
  TestExecutor te(176, 2.0, TestExecutor::LP_SOLVE, SMDPFunctions::LINEARIZED_COST, weights, 180, "inspection_trajectory1.yaml");
//  TestExecutor te(176, 2.0, TestExecutor::LP_SOLVE, SMDPFunctions::LINEARIZED_COST, weights, 180, "inspection_trajectory1.yaml", true, 170);
  TestExecutor te_repeat(176, 2.0, TestExecutor::LP_LOAD, SMDPFunctions::LINEARIZED_COST, weights, 180, "inspection_trajectory1.yaml");
//  vector<double> weights{0.27, -0.34, -0.17, -0.22};
//  TestExecutor te(176, 1.0, TestExecutor::SMDP, SMDPFunctions::LINEARIZED_COST, weights, 180, "inspection_trajectory1.yaml");
//  TestExecutor te_repeat(180, 1.0, TestExecutor::SMDP, SMDPFunctions::LINEARIZED_COST, weights, 180, "inspection_trajectory1.yaml");
//  TestExecutor te(180, 1.0, TestExecutor::MCTS_CONSTRAINED, SMDPFunctions::LINEARIZED_COST, {1, 75, 30}, 60, "iss_trajectory.yaml");
//  TestExecutor te(180, 1.0, TestExecutor::MCTS_SCALARIZED, SMDPFunctions::LINEARIZED_COST,
//      {0.25, -0.25, -0.25, -0.125}, 30, "iss_trajectory.yaml");

//  //This is a temporary return to test the LP solver in isolation
//  return EXIT_SUCCESS;

//  vector<string> trajectory_files = {"experiment_trajectory1.yaml", "experiment_trajectory2.yaml",
//                                     "experiment_trajectory3.yaml", "experiment_trajectory6.yaml",
//                                     "experiment_trajectory5.yaml"};
//
//  vector<double> horizons = {184, 178, 193, 184, 191};

  vector<string> trajectory_files = {"experiment_trajectory1.yaml", "experiment_trajectory2.yaml",
                                     "experiment_trajectory3.yaml", "experiment_trajectory6.yaml",
                                     "experiment_trajectory5.yaml",
                                     "inspection_trajectory1.yaml", "inspection_trajectory2.yaml",
                                     "inspection_trajectory3.yaml", "inspection_trajectory4.yaml",
                                     "inspection_trajectory5.yaml",
                                     "pick_place_trajectory1.yaml", "pick_place_trajectory2.yaml",
                                     "pick_place_trajectory3.yaml", "pick_place_trajectory4.yaml",
                                     "pick_place_trajectory5.yaml"};

  vector<double> horizons = {184, 178, 193, 184, 191,
                             176, 204, 184, 187, 162,
                             172, 174, 189, 165, 181};

  ros::Rate loop_rate(100000000);
//  //TODO: uncomment here for single run
//  te.reset(176, "inspection_trajectory1.yaml", "resolve_testing", false);
//  while (ros::ok())
//  {
//    ros::spinOnce();
////    if (te.run(0.0333333333333))
//    if (te.run(0.0333333333333))
////    if (te_repeat.run(0.01))
//    {
//      break;
//    }
//    loop_rate.sleep();
//  }
//
//  te.reportResults();
//  return EXIT_SUCCESS;

//  //TODO: uncomment here for testing one run on all trajectories
//  vector<double> results_r;
//  vector<double> results_c1;
//  vector<double> results_c2;
//  vector<double> results_c3;
//  results_r.resize(trajectory_files.size());
//  results_c1.resize(trajectory_files.size());
//  results_c2.resize(trajectory_files.size());
//  results_c3.resize(trajectory_files.size());
//
//  for (unsigned int i = 0; i < trajectory_files.size(); i ++)
//  {
//    te.reset(horizons[i], trajectory_files[i]);
//
//    while (ros::ok())
//    {
//      ros::spinOnce();
//      //    if (te.run(0.0333333333333))
//      if (te.run(0.01))
//      {
//        break;
//      }
//      loop_rate.sleep();
//    }
//
//    te.reportResults();
//    results_r[i] += te.r;
//    results_c1[i] += te.c1;
//    results_c2[i] += te.c2;
//    results_c3[i] += te.c3;
//
//    cout << "\n\n\nAll Results:" << endl;
//    cout << "(r, c1, c2, c3)" << endl;
//    for (size_t i = 0; i < results_r.size(); i ++)
//    {
//      cout << results_r[i] << "," << results_c1[i] << "," << results_c2[i] << "," << results_c3[i] << "," << endl;
//    }
//  }
//
//  cout << "\n\n\nFinal Results:" << endl;
//  cout << "(r, c1, c2, c3)" << endl;
//  for (size_t i = 0; i < results_r.size(); i ++)
//  {
//    cout << results_r[i] << "," << results_c1[i] << "," << results_c2[i] << "," << results_c3[i] << "," << endl;
//  }
//
//  cout << "Finished all trials." << endl;

//  //TODO: uncomment here for LP testing
//  vector<double> results_r;
//  vector<double> results_c1;
//  vector<double> results_c2;
//  vector<double> results_c3;
//  results_r.resize(trajectory_files.size()*10);
//  results_c1.resize(trajectory_files.size()*10);
//  results_c2.resize(trajectory_files.size()*10);
//  results_c3.resize(trajectory_files.size()*10);
//
//  int index = -1;
//  for (unsigned int i = 0; i < trajectory_files.size()*10; i ++)
//  {
//    if (i%10 == 0)
//    {
//      index ++;
//      if (!te.reset(horizons[index], trajectory_files[index], trajectory_files[index]))
//      {
//        bool lp_solved = false;
//        for (int attempt = 1; attempt < 3; attempt ++)
//        {
//          lp_solved = te.retryLP(attempt);
//          if (lp_solved)
//          {
//            break;
//          }
//        }
//        if (!lp_solved)
//        {
//          i += 9;
//          cout << "\n\n\nAll Results: (" << index << ")" << endl;
//          cout << "(r, c1, c2, c3)" << endl;
//          for (size_t i = 0; i < results_r.size(); i++)
//          {
//            cout << results_r[i] << "," << results_c1[i] << "," << results_c2[i] << "," << results_c3[i] << "," << endl;
//          }
//          te.freeLP();
//          continue;
//        }
//      }
//    }
//    else
//    {
//      te_repeat.reset(horizons[index], trajectory_files[index], trajectory_files[index]);
//    }
//
//    string log_file;
//    if (i < 50)
//    {
//      log_file = "log_experiment.txt";
//    }
//    else if (i < 100)
//    {
//      log_file = "log_inspection.txt";
//    }
//    else if (i < 150)
//    {
//      log_file = "log_pick_place.txt";
//    }
//
//    while (ros::ok())
//    {
//      ros::spinOnce();
//      //    if (te.run(0.0333333333333))
//      if (i%10 == 0)
//      {
//        if (te.run(0.01, log_file))
//        {
//          break;
//        }
//      }
//      else
//      {
//        if (te_repeat.run(0.01, log_file))
//        {
//          break;
//        }
//      }
//      loop_rate.sleep();
//    }
//
//    if (i % 10 == 0)
//    {
//      te.reportResults();
//      results_r[i] += te.r;
//      results_c1[i] += te.c1;
//      results_c2[i] += te.c2;
//      results_c3[i] += te.c3;
//    }
//    else
//    {
//      te_repeat.reportResults();
//      results_r[i] += te_repeat.r;
//      results_c1[i] += te_repeat.c1;
//      results_c2[i] += te_repeat.c2;
//      results_c3[i] += te_repeat.c3;
//    }
//
//    cout << "\n\n\nAll Results: (" << index << ")" << endl;
//    cout << "(r, c1, c2, c3)" << endl;
//    for (size_t i = 0; i < results_r.size(); i ++)
//    {
//      cout << results_r[i] << "," << results_c1[i] << "," << results_c2[i] << "," << results_c3[i] << "," << endl;
//    }
//  }
//
//  cout << "\n\n\nFinal Results:" << endl;
//  cout << "(r, c1, c2, c3)" << endl;
//  for (size_t i = 0; i < results_r.size(); i ++)
//  {
//    cout << results_r[i] << "," << results_c1[i] << "," << results_c2[i] << "," << results_c3[i] << "," << endl;
//  }
//
//  cout << "Finished all trials." << endl;

  //TODO: uncomment here for logging LP solutions over varied data
  vector<double> results_r;
  vector<double> results_c1;
  vector<double> results_c2;
  vector<double> results_c3;

  ros::Time start_time = ros::Time::now();
  for (size_t trials = 0; trials < 16; trials ++)
  {
    ROS_INFO("Start of trial %d", trials);
    results_r.resize(results_r.size() + trajectory_files.size());
    results_c1.resize(results_c1.size() + trajectory_files.size());
    results_c2.resize(results_c2.size() + trajectory_files.size());
    results_c3.resize(results_c3.size() + trajectory_files.size());

    for (unsigned int i = 0; i < trajectory_files.size(); i++)
    {
      te.randomizeWeights();
      if (!te.reset(horizons[i], trajectory_files[i], trajectory_files[i], true, true))
      {
        bool lp_solved = false;
        for (int attempt = 1; attempt < 3; attempt++)
        {
          lp_solved = te.retryLP(attempt);
          if (lp_solved)
          {
            break;
          }
        }
        if (!lp_solved)
        {
          cout << "\n\n\nAll Results: (" << i << " x" << trials + 1 << ")" << endl;
          cout << "(r, c1, c2, c3)" << endl;
          for (size_t j = 0; j < results_r.size(); j++)
          {
            cout << results_r[j] << "," << results_c1[j] << "," << results_c2[j] << "," << results_c3[j] << "," << endl;
          }
          te.freeLP();
          continue;
        }
      }

      string log_file;
      if (i < 5)
      {
        log_file = "log_experiment.txt";
      }
      else if (i < 10)
      {
        log_file = "log_inspection.txt";
      }
      else if (i < 15)
      {
        log_file = "log_pick_place.txt";
      }

      while (ros::ok())
      {
        ros::spinOnce();
        //    if (te.run(0.0333333333333))
        if (te.run(0.01, log_file))
        {
          break;
        }
        loop_rate.sleep();
      }

      te.reportResults();
      results_r[trials*trajectory_files.size() + i] += te.r;
      results_c1[trials*trajectory_files.size() + i] += te.c1;
      results_c2[trials*trajectory_files.size() + i] += te.c2;
      results_c3[trials*trajectory_files.size() + i] += te.c3;

      cout << "\n\n\nAll Results: (" << i << " x" << trials + 1 << ")" << endl;
      cout << "(r, c1, c2, c3)" << endl;
      for (size_t j = 0; j < results_r.size(); j++)
      {
        cout << results_r[j] << "," << results_c1[j] << "," << results_c2[j] << "," << results_c3[j] << "," << endl;
      }
    }
  }

  cout << "\n\n\nFinal Results:" << endl;
  cout << "(r, c1, c2, c3)" << endl;
  for (size_t j = 0; j < results_r.size(); j ++)
  {
    cout << results_r[j] << "," << results_c1[j] << "," << results_c2[j] << "," << results_c3[j] << "," << endl;
  }

  cout << "Finished all trials." << endl;
  cout << "Full duration: " << (ros::Time::now() - start_time).toSec() << endl;

  return EXIT_SUCCESS;
}
