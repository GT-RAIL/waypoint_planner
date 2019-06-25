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

TestExecutor::TestExecutor(double horizon, double step, uint8_t approach, uint8_t mode, vector<double> weights,
    bool optimal) :
    solver(horizon, step, mode, "iss_waypoints.csv", weights),    // TODO: parameters here for optional values
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
  weights[1] = 50*static_cast<double>(rand())/RAND_MAX + 10;
  weights[2] = 50*static_cast<double>(rand())/RAND_MAX + 20;

  cout << "New weights: " << weights[0] << ", " << weights[1] << ", " << weights[2] << endl;
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
    solver.backwardsInduction();
    ROS_INFO("Policy computed.");
  }

  return true;
}

bool TestExecutor::run(double sim_step, bool vis)
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

    geometry_msgs::Point goal;
    if (current_action.actionType() == Action::MOVE)
    {
      ROS_INFO("Starting move action.");
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
      ROS_INFO("Starting perch action.");
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
      ROS_INFO("Starting unperch action.");
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

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_executor");

  vector<double> weights{1, -.333, -.333, -.333};

  // initialize a solver over a trajectory distribution, and a best-case evaluation solver (i.e. trajectory known)
  TestExecutor te(180, 1.0, TestExecutor::SMDP, SMDPFunctions::LINEARIZED_COST, weights);
  TestExecutor te_eval(180, 1.0, TestExecutor::SMDP, SMDPFunctions::LINEARIZED_COST, weights, true);

  // set up containers for trajectory sampling
  vector<HumanTrajectory> trajectories;
  HumanTrajectory eval_trajectory;
  size_t num_trajectory_samples = 100;

  // set up list of tasks
  vector<string> trajectory_seeds = {"experiment_times.yaml", "inspection_times.yaml", "pick_place_times.yaml"};
  vector<double> horizons = {180, 180, 180};

  ros::Rate loop_rate(100);

  // set a new task
  string trajectory_file = trajectory_seeds[2];
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
  te.solve();
  te_eval.solve();

  while (ros::ok())
  {
    ros::spinOnce();
    bool te_finished = te.run(0.0333333333333, true);
    bool te_eval_finished = te_eval.run(0.0333333333333, true);
    if (te_finished || te_eval_finished)
    {
      break;
    }
    loop_rate.sleep();
  }

  ROS_INFO("Results solved over distribution: ");
  te.reportResults();
  cout << endl;
  ROS_INFO("Best case results: ");
  te_eval.reportResults();

  return EXIT_SUCCESS;

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
//
//  return EXIT_SUCCESS;
}
