#include "waypoint_planner/TestExecutorDist.h"

using std::vector;
using std::cout;
using std::endl;
using std::string;

const uint8_t TestExecutor::SMDP = 0;
const uint8_t TestExecutor::LP_SOLVE = 1;
const uint8_t TestExecutor::LP_LOAD = 2;
const uint8_t TestExecutor::MCTS_CONSTRAINED = 3;
const uint8_t TestExecutor::MCTS_SCALARIZED = 4;
const uint8_t TestExecutor::RANDOM = 5;
const uint8_t TestExecutor::REACTIVE = 6;
const uint8_t TestExecutor::APPROXIMATE_STOCHASTIC = 7;
const uint8_t TestExecutor::APPROXIMATE_DETERMINISTIC = 8;

TestExecutor::TestExecutor(double horizon, double step, uint8_t approach, uint8_t mode, vector<double> weights,
    bool optimal) :
    solver(horizon, step, mode, "iss_waypoints.csv", weights),
    lp_solver(horizon, step),
    reactive_baseline("iss_waypoints.csv", weights),
    random_baseline("iss_waypoints.csv"),
    current_action(Action::OBSERVE),
    pnh("~")
{
  srand(time(NULL));

  this->approach = approach;
  this->mode = mode;

  this->optimal = optimal;

  this->weights.resize(weights.size());
  for (size_t i = 0; i < weights.size(); i ++)
  {
    this->weights[i] = weights[i];
  }

  default_human_dims.x = 0.5;
  default_human_dims.y = 0.4;
  default_human_dims.z = 1.4;

  time_horizon = horizon;
  current_time = 0;
  next_decision = 0;
  time_step = step;

  // TODO: better state initialization (currently just takes the first state)
  state.waypoint.x = 11.39;
  state.waypoint.y = -10.12;
  state.waypoint.z = 4.45;
  state.perched = false;

  r = 0;
  c1 = 0;
  c2 = 0;
  c3 = 0;

  if (optimal)
  {
    optimal_robot_vis_publisher = pnh.advertise<visualization_msgs::Marker>("test_optimal_robot_vis", 1, this);
  }
  else
  {
    robot_vis_publisher = pnh.advertise<visualization_msgs::Marker>("test_robot_vis", 1, this);
  }
  human_sim_time_publisher = n.advertise<std_msgs::Float32>("human_simulator/time_update", 1, this);
  human_trajectory_publisher = n.advertise<waypoint_planner::HumanTrajectoryMsg>("human_simulator/trajectory_update", 1,
      this);

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
  if (optimal)
  {
    robot_marker.color.a /= 2;
  }
}

void TestExecutor::randomizeWeights()
{
  weights[0] = 4*static_cast<double>(rand())/RAND_MAX + 1;
  weights[1] = 80*static_cast<double>(rand())/RAND_MAX + 10;
  weights[2] = 80*static_cast<double>(rand())/RAND_MAX + 20;

  cout << "New weights: " << weights[0] << ", " << weights[1] << ", " << weights[2] << endl;
}

void TestExecutor::setWeights(std::vector<double> weights)
{
  this->weights.resize(weights.size());
  for (size_t i = 0; i < weights.size(); i ++)
  {
    this->weights[i] = weights[i];
  }
}

void TestExecutor::reset(double horizon)
{
  if (approach == TestExecutor::SMDP)
  {
    solver.reset(horizon);
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
  if (optimal)
  {
    robot_marker.color.a /= 2;
  }
}

void TestExecutor::setTrajectories(std::vector<HumanTrajectory> trajectories)
{
  this->trajectories = trajectories;
  if (approach == TestExecutor::SMDP)
  {
    solver.setTrajectory(trajectories);
  }
  else if (approach == TestExecutor::LP_SOLVE || TestExecutor::LP_LOAD)
  {
    lp_solver.setTrajectory(trajectories);
  }
}

void TestExecutor::setEvalTrajectory(HumanTrajectory eval_trajectory)
{
  this->eval_trajectory = eval_trajectory;
  human_trajectory_publisher.publish(eval_trajectory.toMsg());
}

bool TestExecutor::solve()
{
  if (this->approach == SMDP)
  {
    solver.setWeights(weights);
    solver.backwardsInduction();
    ROS_INFO("Policy computed.");
  }
  else if (this->approach == LP_SOLVE)
  {
    lp_solver.constructModel(weights);
    // try some different parameters
    bool lp_solved = false;
    for (int attempt = 0; attempt < 4; attempt ++)
    {
      lp_solver.setScaling(attempt);  // TODO: this is only here for scaling mode testing
//        lp_solver.setScaling(attempt);
      ros::Time start_time = ros::Time::now();
      lp_solved = lp_solver.solveModel(360);
      if (lp_solved)
      {
        std::ofstream log_file;
        log_file.open("solve_times.txt", std::ios::out | std::ios::app);
        log_file << "\tSolve time: " << (ros::Time::now() - start_time).toSec() << "; scaling mode: " << attempt << endl;
        log_file.close();
        cout << "Solve time: " << (ros::Time::now() - start_time).toSec() << endl;
        break;
      }
    }
    if (!lp_solved)
    {
      ROS_INFO("Could not solve LP.");
      return false;
    }
  }
  else if (this->approach == LP_LOAD)
  {
    lp_solver.loadModel("var_results.txt");
    ROS_INFO("Model loaded.");
  }

  return true;
}

int TestExecutor::run(double sim_step, bool vis, bool log_policy, string log_name)
{
  if (current_time >= next_decision)
  {
    if (current_action.actionType() == Action::MOVE)
    {
//      ROS_INFO("Move action complete.");
      state.waypoint = current_action.actionGoal();
      robot_marker.pose.position = state.waypoint;
      robot_marker.color.r = 1.0;
      robot_marker.color.g = 0.0;
      robot_marker.color.b = 1.0;
      robot_marker.color.a = 1.0;
      if (optimal)
      {
        robot_marker.color.a /= 2;
      }
    }
    else if (current_action.actionType() == Action::PERCH)
    {
      state.perched = true;
      robot_marker.color.r = 0.0;
      robot_marker.color.g = 1.0;
      robot_marker.color.b = 1.0;
      robot_marker.color.a = 1.0;
      if (optimal)
      {
        robot_marker.color.a /= 2;
      }
    }
    else if (current_action.actionType() == Action::UNPERCH)
    {
      state.perched = false;
      robot_marker.color.r = 1.0;
      robot_marker.color.g = 0.0;
      robot_marker.color.b = 1.0;
      robot_marker.color.a = 1.0;
      if (optimal)
      {
        robot_marker.color.a /= 2;
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
    else if (approach == RANDOM)
    {

    }
    else if (approach == REACTIVE)
    {

    }

//    ROS_INFO("1");

    // error case, need to wait for next time step to get an action
    if (current_action.actionType() == Action::NO_ACTION)
    {
      // determine duration to next time step
      double next_time = floor(current_time + 1.0);
      double wait_duration = next_time - current_time;

      // update rewards/costs for extra wait time
      double r0 = RewardsAndCosts::reward_recognition(eval_trajectory.getPose(current_time),
                                                      default_human_dims, state.waypoint)*wait_duration;
      double c1_0 = RewardsAndCosts::cost_collision(eval_trajectory.getPose(current_time), default_human_dims,
                                                    state.waypoint)*wait_duration;
      double c2_0 = RewardsAndCosts::cost_intrusion(eval_trajectory.getPose(current_time), state.waypoint, state.perched)*wait_duration;
      double c3_0 = RewardsAndCosts::cost_power(state.perched, current_action)*wait_duration;

      // calculate rewards and costs and add them to the totals
      r += r0;
      c1 += c1_0;
      c2 += c2_0;
      c3 += c3_0;

      // update current time and re-do action selection
      current_time = next_time;
      if (approach == SMDP)
      {
        current_action = solver.getAction(state, current_time);
      }
      else if (approach == LP_SOLVE || approach == LP_LOAD)
      {
        current_action = lp_solver.getAction(state, current_time);
      }

      // if there's still an error, return a failure
      if (current_action.actionType() == Action::NO_ACTION)
      {
        return -1;
      }
    }

    geometry_msgs::Point goal;
    if (current_action.actionType() == Action::MOVE)
    {
//      ROS_INFO("Starting move action.");
      goal = current_action.actionGoal();
      robot_marker.color.r = 1.0;
      robot_marker.color.g = 0.5;
      robot_marker.color.b = 0.0;
      robot_marker.color.a = 0.6;
      if (optimal)
      {
        robot_marker.color.a /= 2;
      }
    }
    else if (current_action.actionType() == Action::PERCH)
    {
//      ROS_INFO("Starting perch action.");
      goal = state.waypoint;
      robot_marker.color.r = 0.0;
      robot_marker.color.g = 1.0;
      robot_marker.color.b = 1.0;
      robot_marker.color.a = 0.6;
      if (optimal)
      {
        robot_marker.color.a /= 2;
      }
    }
    else if (current_action.actionType() == Action::UNPERCH)
    {
//      ROS_INFO("Starting unperch action.");
      goal = state.waypoint;
      robot_marker.color.r = 1.0;
      robot_marker.color.g = 0.0;
      robot_marker.color.b = 1.0;
      robot_marker.color.a = 0.6;
      if (optimal)
      {
        robot_marker.color.a /= 2;
      }
    }
    else
    {
//      ROS_INFO("Observing.");
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
//    ROS_INFO("Action duration: %f", duration);

    double r0 = RewardsAndCosts::reward_recognition(eval_trajectory.getPose(current_time),
        default_human_dims, state.waypoint)*duration;
    double c1_0 = RewardsAndCosts::cost_collision(eval_trajectory.getPose(current_time), default_human_dims,
        state.waypoint)*duration;
    double c2_0 = RewardsAndCosts::cost_intrusion(eval_trajectory.getPose(current_time), state.waypoint, state.perched)*duration;
    double c3_0 = RewardsAndCosts::cost_power(state.perched, current_action)*duration;

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

//    ROS_INFO("Logging policy...");
    if (log_policy)
    {
      // log data, in the form:
      //  action type, goal x, goal y, goal z, c1 remaining, c2 remaining, c3 remaining, current x, current y, current z,
      //  perched, time remaining, [remaining human trajectory: x, y, z, qx, qy, qz, qw, ...]
      std::stringstream traj_stream;
      for (double t_remaining = current_time; t_remaining < time_horizon; t_remaining += time_step)
      {
        geometry_msgs::Pose traj_remaining_pose = eval_trajectory.getPose(t_remaining);
        traj_stream << traj_remaining_pose.position.x << "," << traj_remaining_pose.position.y << ","
                    << traj_remaining_pose.position.z << "," << traj_remaining_pose.orientation.x << ","
                    << traj_remaining_pose.orientation.y << "," << traj_remaining_pose.orientation.z << ","
                    << traj_remaining_pose.orientation.w << ",";
      }
//      ROS_INFO("2");
      std::ofstream log_file;
      log_file.open(log_name, std::ios::out | std::ios::app);
      log_file << std::to_string(current_action.actionType()) << "," << current_action.actionGoal().x << ","
               << current_action.actionGoal().y << "," << current_action.actionGoal().z << "," << weights[0] - c1 << ","
               << weights[1] - c2 << "," << weights[2] - c3 << "," << state.waypoint.x << "," << state.waypoint.y << ","
               << state.waypoint.z << "," << state.perched << "," << time_horizon - current_time << ","
               << traj_stream.str() << endl;
      log_file.close();
    }

//    std::cout << "Time: " << current_time << "\tReward: " << r << ", C1: " << c1 << ", C2: " << c2 << ", C3: " << c3
//      << std::endl;
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

  if (vis && !optimal)
  {
    // send time update to human trajectory visualizer
    robot_vis_publisher.publish(robot_marker);
    std_msgs::Float32 change_time;
    change_time.data = current_time;
    human_sim_time_publisher.publish(change_time);
  }
  else if (vis && optimal)
  {
    optimal_robot_vis_publisher.publish(robot_marker);
  }

  if (current_time > time_horizon)
  {
    return 1;
  }
  return 0;
}

void TestExecutor::reportResults()
{
  std::cout << "Trial complete." << std::endl;
  std::cout << "Total accumulated reward: " << r << std::endl;
  std::cout << "Total accumulated collision cost: " << c1 << std::endl;
  std::cout << "Total accumulated intrusion cost: " << c2 << std::endl;
  std::cout << "Total accumulated power cost: " << c3 << std::endl;
}

void collectSolveTimes()
{
  vector<double> weights{1, 40, 40};
  TestExecutor te(180, 1.0, TestExecutor::LP_SOLVE, SMDPFunctions::LINEARIZED_COST, weights);
  TestExecutor te_eval(180, 1.0, TestExecutor::LP_SOLVE, SMDPFunctions::LINEARIZED_COST, weights, true);

  // set up containers for trajectory sampling
  vector<HumanTrajectory> trajectories;
  HumanTrajectory eval_trajectory;
  size_t num_trajectory_samples = 10;

  // set up list of tasks
  vector<string> trajectory_seeds = {"experiment_times.yaml", "inspection_times.yaml", "pick_place_times.yaml"};
  vector<double> horizons = {180, 180, 180};

  ros::Rate loop_rate(100000);

  while (true)
  {
    // set a new task
    string trajectory_file = trajectory_seeds[floor(3.0 * static_cast<double>(rand()) / RAND_MAX)];
    te.reset(horizons[0]);
    te_eval.reset(horizons[0]);

    std::ofstream log_file;
    log_file.open("solve_times.txt", std::ios::out | std::ios::app);
    log_file << "\n\n" << trajectory_file << endl;
    log_file.close();

    // sample a new set of trajectories, and an evaluation trajectory
    string trajectory_file_path = ros::package::getPath("waypoint_planner") + "/config/" + trajectory_file;
    trajectories.clear();
    EnvironmentSetup::sampleHumanTrajectories(trajectory_file_path, trajectories, num_trajectory_samples);
    vector<HumanTrajectory> eval_samples;
    EnvironmentSetup::sampleHumanTrajectories(trajectory_file_path, eval_samples, 1);
    eval_trajectory = eval_samples[0];

    // update solver with new trajectories
    te.setTrajectories(trajectories);
    te.setEvalTrajectory(eval_trajectory);
    te_eval.setTrajectories(eval_samples);
    te_eval.setEvalTrajectory(eval_trajectory);

    // solve for policy
    if (!te.solve())
    {
      ROS_INFO("Solve failed for multiple sampled trajectories");
    }
    if (!te_eval.solve())
    {
      ROS_INFO("Solve failed for single sampled trajectory");
    }
  }
}

void testSingleCase(int rate, bool tuning_mode)
{
  // Scalarization
  vector<double> weights{0.27, -0.34, -0.17, -0.22};
  TestExecutor te(180, 1.0, TestExecutor::SMDP, SMDPFunctions::LINEARIZED_COST, weights);
  TestExecutor te_eval(180, 1.0, TestExecutor::SMDP, SMDPFunctions::LINEARIZED_COST, weights, true);

//  // CMDP
//  vector<double> weights{1, 20, 40};
//  TestExecutor te(180, 1.0, TestExecutor::LP_SOLVE, SMDPFunctions::LINEARIZED_COST, weights);
//  TestExecutor te_eval(180, 1.0, TestExecutor::LP_SOLVE, SMDPFunctions::LINEARIZED_COST, weights, true);

  // set up containers for trajectory sampling
  vector<HumanTrajectory> trajectories;
  HumanTrajectory eval_trajectory;
  size_t num_trajectory_samples = 25;

  // set up list of tasks
  vector<string> trajectory_seeds = {"experiment_times.yaml", "inspection_times.yaml", "pick_place_times.yaml"};
  vector<double> horizons = {180, 180, 180};

  // set a new task
  string trajectory_file = trajectory_seeds[1];
  te.reset(horizons[0]);
  te_eval.reset(horizons[0]);

  // sample a new set of trajectories, and an evaluation trajectory
  string trajectory_file_path = ros::package::getPath("waypoint_planner") + "/config/" + trajectory_file;
  trajectories.clear();
  EnvironmentSetup::sampleHumanTrajectories(trajectory_file_path, trajectories, num_trajectory_samples);
  vector<HumanTrajectory> eval_samples;
  EnvironmentSetup::sampleHumanTrajectories(trajectory_file_path, eval_samples, 1);
  eval_trajectory = eval_samples[0];

  // update solver with new trajectories
  te.setTrajectories(trajectories);
  te.setEvalTrajectory(eval_trajectory);
  te_eval.setTrajectories(eval_samples);
  te_eval.setEvalTrajectory(eval_trajectory);

  // solve for policy
  if (!te.solve())
  {
    ROS_INFO("Solve failed for multiple sampled trajectories");
    return;
  }
  if (!tuning_mode)
  {
    if (!te_eval.solve())
    {
      ROS_INFO("Solve failed for single sampled trajectory");
      return;
    }
  }

  ROS_INFO("Solving complete, provide input to start visualization...");
  string str;
  std::cin >> str;

  ros::Rate loop_rate(rate);

  while (ros::ok())
  {
    ros::spinOnce();
    int te_eval_result = 0;
    int te_result = te.run(0.0333333333333, true);
    if (!tuning_mode)
    {
      te_eval_result = te_eval.run(0.0333333333333, true);
    }
    if (te_result == -1 || te_eval_result == -1)
    {
      ROS_INFO("An action selection error occurred.  Terminating run.");
      break;
    }
    if (te_result == 1 || te_eval_result == 1)
    {
      break;
    }
    loop_rate.sleep();
  }

  ROS_INFO("Results solved over distribution: ");
  te.reportResults();
  cout << endl;
  if (!tuning_mode)
  {
    ROS_INFO("Best case results: ");
    te_eval.reportResults();
  }
}

void testFullSet(int runs_per_task, int num_trajectory_samples)
{
  // Scalarization
  vector<double> weights1{0.27, -0.34, -0.17, -0.22};
  vector<double> weights2{0.35, -0.43, -0.22, 0.0};
  vector<double> weights3{0.33, -0.41, 0.0, -0.26};
  vector<double> weights4{0.67, -0.33, 0.0, 0.0};
  vector< vector<double> > weights_scal = {weights1, weights2, weights3, weights4};
//  vector< vector<double> > weights_scal = {weights1, weights3};
  TestExecutor te(180, 1.0, TestExecutor::SMDP, SMDPFunctions::LINEARIZED_COST, weights_scal[0]);
  TestExecutor te_eval(180, 1.0, TestExecutor::SMDP, SMDPFunctions::LINEARIZED_COST, weights_scal[0], true);

  // CMDP
  vector<double> cweights1{1, 20, 40};
  vector<double> cweights2{1, 20, 180};
  vector<double> cweights3{1, 180, 40};
  vector<double> cweights4{1, 180, 180};
  vector< vector<double> > weights_cmdp = {cweights1, cweights2, cweights3, cweights4};
//  vector< vector<double> > weights_cmdp = {cweights1, cweights3};
  TestExecutor te_cmdp(180, 1.0, TestExecutor::LP_SOLVE, SMDPFunctions::LINEARIZED_COST, weights_cmdp[0]);
  TestExecutor te_cmdp_eval(180, 1.0, TestExecutor::LP_SOLVE, SMDPFunctions::LINEARIZED_COST, weights_cmdp[0], true);

  // set up containers for trajectory sampling
  vector<HumanTrajectory> trajectories;
  HumanTrajectory eval_trajectory;

  // set up list of tasks
  vector<string> trajectory_seeds = {"experiment_times.yaml", "inspection_times.yaml", "pick_place_times.yaml"};
  vector<double> horizons = {180, 180, 180};

  for (size_t w = 0; w < weights_scal.size(); w ++)
  {
    for (size_t i = 0; i < trajectory_seeds.size(); i ++)
    {
      if (i == 0)
        continue;
      if (i == 1 && w != 1)
        continue;
      if (i == 2 && w == 0)
        continue;
      std::ofstream log_file;
      log_file.open("results_scal-" + std::to_string(i) + ".txt", std::ios::out | std::ios::app);
      log_file << "----- Weight Set " << std::to_string(w) << " -----" << endl;
      log_file.close();
      log_file.open("results_cmdp-" + std::to_string(i) + ".txt", std::ios::out | std::ios::app);
      log_file << "----- Weight Set " << std::to_string(w) << " -----" << endl;
      log_file.close();
      te.setWeights(weights_scal[w]);
      te_eval.setWeights(weights_scal[w]);
      te_cmdp.setWeights(weights_cmdp[w]);
      te_cmdp_eval.setWeights(weights_cmdp[w]);
    }
    for (size_t i = 0; i < trajectory_seeds.size(); i ++)
    {
      if (i == 0)
        continue;
      if (i == 1 && w != 1)
        continue;
      if (i == 2 && w == 0)
        continue;
      for (size_t j = 0; j < runs_per_task; j++)
      {
        // set a new task
        string trajectory_file = trajectory_seeds[i];
        te.reset(horizons[0]);
        te_eval.reset(horizons[0]);
        te_cmdp.reset(horizons[0]);
        te_cmdp_eval.reset(horizons[0]);

        // sample a new set of trajectories, and an evaluation trajectory
        string trajectory_file_path = ros::package::getPath("waypoint_planner") + "/config/" + trajectory_file;
        trajectories.clear();
        EnvironmentSetup::sampleHumanTrajectories(trajectory_file_path, trajectories, num_trajectory_samples);
        vector<HumanTrajectory> eval_samples;
        EnvironmentSetup::sampleHumanTrajectories(trajectory_file_path, eval_samples, 1);
        eval_trajectory = eval_samples[0];

        // update solver with new trajectories
        te.setTrajectories(trajectories);
        te.setEvalTrajectory(eval_trajectory);
        te_eval.setTrajectories(eval_samples);
        te_eval.setEvalTrajectory(eval_trajectory);
        te_cmdp.setTrajectories(trajectories);
        te_cmdp.setEvalTrajectory(eval_trajectory);
        te_cmdp_eval.setTrajectories(eval_samples);
        te_cmdp_eval.setEvalTrajectory(eval_trajectory);

        // solve for policy
        if (!te_cmdp.solve())
        {
          ROS_INFO("Solve failed for multiple sampled trajectories (CMDP)");
          continue;
        }
        if (!te_cmdp_eval.solve())
        {
          ROS_INFO("Solve failed for single sampled trajectory (CMDP)");
          continue;
        }
        if (!te.solve())
        {
          ROS_INFO("Solve failed for multiple sampled trajectories");
          continue;
        }
        if (!te_eval.solve())
        {
          ROS_INFO("Solve failed for single sampled trajectory");
          continue;
        }

        ros::Rate loop_rate(100000000);

        bool action_selection_failure = false;
        while (ros::ok())
        {
          ros::spinOnce();
          int te_result = te.run(0.0333333333333, false);
          int te_eval_result = te_eval.run(0.0333333333333, false);
          int te_cmdp_result = te_cmdp.run(0.0333333333333, false);
          int te_cmdp_eval_result = te_cmdp_eval.run(0.0333333333333, false);
          if ((te_result == -1 || te_eval_result == -1) && (te_cmdp_result == -1 || te_cmdp_eval_result == -1))
          {
            ROS_INFO("An action selection error has occurred.  Terminating run.");
            action_selection_failure = true;
            break;
          }
          if ((te_result == 1 || te_eval_result == 1) && (te_cmdp_result == 1 || te_cmdp_eval_result == 1))
          {
            break;
          }
          loop_rate.sleep();
        }

        if (action_selection_failure)
        {
          continue;
        }

        ROS_INFO("Results solved over distribution (CMDP): ");
        te_cmdp.reportResults();
        cout << endl;
        ROS_INFO("Best case results (CMDP): ");
        te_cmdp_eval.reportResults();
        ROS_INFO("Best case results: ");
        te_eval.reportResults();
        ROS_INFO("Results solved over distribution: ");
        te.reportResults();
        cout << endl;
        ROS_INFO("Best case results: ");
        te_eval.reportResults();

        std::ofstream log_file;
        log_file.open("results_scal-" + std::to_string(i) + ".txt", std::ios::out | std::ios::app);
        log_file << te.r << ", " << te.c1 << ", " << te.c2 << ", " << te.c3 << "; " << te_eval.r << ", " << te_eval.c1
                 << ", " << te_eval.c2 << ", " << te_eval.c3 << endl;
        log_file.close();
        log_file.open("results_cmdp-" + std::to_string(i) + ".txt", std::ios::out | std::ios::app);
        log_file << te_cmdp.r << ", " << te_cmdp.c1 << ", " << te_cmdp.c2 << ", " << te_cmdp.c3 << "; "
          << te_cmdp_eval.r << ", " << te_cmdp_eval.c1 << ", " << te_cmdp_eval.c2 << ", " << te_cmdp_eval.c3 << endl;
        log_file.close();
      }
    }
  }
}

void logPolicyData(bool random_traj)
{
  // CMDP
  vector<double> cweights{1, 20, 40};
  TestExecutor te_cmdp(180, 1.0, TestExecutor::LP_SOLVE, SMDPFunctions::LINEARIZED_COST, cweights);

  // set up containers for trajectory sampling
  HumanTrajectory eval_trajectory;

  // set up list of tasks
  vector<string> trajectory_seeds = {"experiment_times", "inspection_times", "pick_place_times"};
  vector<double> horizons = {180, 180, 180};

  for (size_t run_counter = 0; run_counter < 1500000; run_counter ++)
  {
    for (size_t i = 0; i < trajectory_seeds.size(); i ++)
    {
      te_cmdp.randomizeWeights();

      // set a new task
      string trajectory_file = trajectory_seeds[i] + ".yaml";
      te_cmdp.reset(horizons[0]);

      // sample a new set of trajectories, and an evaluation trajectory
      string trajectory_file_path = ros::package::getPath("waypoint_planner") + "/config/" + trajectory_file;
      vector<HumanTrajectory> eval_samples;
      if (random_traj)
      {
        eval_samples.resize(1);
        for (size_t i = 0; i < eval_samples.size(); i ++)
        {
          eval_samples[i].sampleRandomTrajectory(180);
          eval_samples[i].splineTrajectory(0.0333);
        }
      }
      else
      {
        EnvironmentSetup::sampleHumanTrajectories(trajectory_file_path, eval_samples, 1, 0, false);
        for (size_t i = 0; i < eval_samples.size(); i ++)
        {
          eval_samples[i].perturbTrajectory();
          eval_samples[i].splineTrajectory(0.0333);
        }
      }
      eval_trajectory = eval_samples[0];

      // update solver with new trajectories
      te_cmdp.setTrajectories(eval_samples);
      te_cmdp.setEvalTrajectory(eval_trajectory);

      // solve for policy
      if (!te_cmdp.solve())
      {
        ROS_INFO("Solve failed for sampled trajectory (CMDP)");
        continue;
      }

      ros::Rate loop_rate(100000000);

      bool action_selection_failure = false;
      while (ros::ok())
      {
        ros::spinOnce();
        int te_cmdp_result;
        if (random_traj)
        {
          te_cmdp_result = te_cmdp.run(0.0333333333333, false, true,"log_random.csv");
        }
        else
        {
          te_cmdp_result = te_cmdp.run(0.0333333333333, false, true, trajectory_seeds[i] + ".txt");
        }
        if (te_cmdp_result == -1)
        {
          ROS_INFO("An action selection error has occurred.  Terminating run.");
          action_selection_failure = true;
          break;
        }
        if (te_cmdp_result == 1)
        {
          break;
        }
        loop_rate.sleep();
      }

      if (action_selection_failure)
      {
        std::ofstream log_file;
        if (random_traj)
        {
          log_file.open("log_random.csv", std::ios::out | std::ios::app);
        }
        else
        {
          log_file.open(trajectory_seeds[i] + ".txt", std::ios::out | std::ios::app);
        }
        log_file << "---- Action Selection Failure ----" << endl;
        log_file.close();
        continue;
      }

      ROS_INFO("Results solved over sampled trajectory (CMDP): ");
      te_cmdp.reportResults();
      cout << endl;

      std::ofstream log_file;
      if (random_traj)
      {
        log_file.open("log_random.csv", std::ios::out | std::ios::app);
      }
      else
      {
        log_file.open(trajectory_seeds[i] + ".txt", std::ios::out | std::ios::app);
      }
      log_file << "----------------------------------" << endl;
      log_file.close();
    }
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_executor");

//  collectSolveTimes();

//  testSingleCase();
//  testSingleCase(100, true);

//  testFullSet(4, 25);

  logPolicyData(true);

  return EXIT_SUCCESS;
}
