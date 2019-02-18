#include "waypoint_planner/TestExecutor.h"

using std::vector;
using std::cout;
using std::endl;

const uint8_t TestExecutor::SMDP = 0;
const uint8_t TestExecutor::LP_SOLVE = 1;
const uint8_t TestExecutor::LP_LOAD = 2;
const uint8_t TestExecutor::MCTS_CONSTRAINED = 3;
const uint8_t TestExecutor::MCTS_SCALARIZED = 4;

TestExecutor::TestExecutor(double horizon, double step, uint8_t approach, uint8_t mode, vector<double> weights,
    size_t search_depth) :
    solver(horizon, step, mode, "iss_trajectory.yaml", "iss_waypoints.csv", weights),    // TODO: parameters here for optional values
    lp_solver(horizon, step, "iss_trajectory.yaml", "iss_waypoints.csv"),    // TODO: parameters here for optional values
//    mcts_solver(horizon, step, "iss_trajectory.yaml", "iss_waypoints.csv", {1.0, 75.0}, 150.0,
//        static_cast<size_t>(horizon/step), 2.0),  // TODO: parameters here for optional values
    mcts_reward_solver(horizon, step, "iss_trajectory.yaml", "iss_waypoints.csv", weights, 30.0,
        search_depth, 2, 6),
    mcts_scalarized_solver(horizon, step, "iss_trajectory.yaml", "iss_waypoints.csv", weights, 120.0,
        search_depth, 10000, 6),
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

  std::string trajectory_file_path = ros::package::getPath("waypoint_planner") + "/config/iss_trajectory.yaml";
  trajectory = EnvironmentSetup::readHumanTrajectory(trajectory_file_path);

  default_human_dims.x = 0.5;
  default_human_dims.y = 0.4;
  default_human_dims.z = 1.4;

  if (this->approach == LP_SOLVE)
  {
    lp_solver.constructModel(weights);  // constraint thresholds {d1, d2, d3} packed into weights
    lp_solver.solveModel(600);  // solver timeout (s) before restarting
    ROS_INFO("LP model solved.");
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
  robot_marker.scale.x = 0.2;
  robot_marker.scale.y = 0.2;
  robot_marker.scale.z = 0.2;
  robot_marker.color.r = 1.0;
  robot_marker.color.g = 0.0;
  robot_marker.color.b = 1.0;
  robot_marker.color.a = 1.0;
}

bool TestExecutor::run(double sim_step)
{
  if (current_time >= next_decision)
  {
    if (current_action.actionType() == Action::MOVE)
    {
      ROS_INFO("Move action complete.");
      state.waypoint = current_action.actionGoal();
      robot_marker.pose.position = state.waypoint;
      robot_marker.color.a = 1.0;
    }
    else if (current_action.actionType() == Action::PERCH)
    {
      state.perched = true;
      robot_marker.color.r = 0.0;
      robot_marker.color.g = 1.0;
      robot_marker.color.a = 1.0;
    }
    else if (current_action.actionType() == Action::UNPERCH)
    {
      state.perched = false;
      robot_marker.color.r = 1.0;
      robot_marker.color.g = 0.0;
      robot_marker.color.a = 1.0;
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
      robot_marker.color.a = 0.5;
    }
    else if (current_action.actionType() == Action::PERCH)
    {
      ROS_INFO("Starting perch action.");
      goal = state.waypoint;
      robot_marker.color.a = 0.5;
    }
    else if (current_action.actionType() == Action::UNPERCH)
    {
      ROS_INFO("Starting unperch action.");
      goal = state.waypoint;
      robot_marker.color.a = 0.5;
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

    std::cout << "Time: " << current_time << "\tReward: " << r << ", C1: " << c1 << ", C2: " << c2 << ", C3: " << c3
      << std::endl;
  }

  // update fake execution time
  current_time += sim_step;

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

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_executor");
  vector<double> weights{0.25, -0.25, -0.25, -0.125};
//  TestExecutor te(150, 1.0, TestExecutor::LP_SOLVE, SMDPFunctions::LINEARIZED_COST, {1.0, 75.0, 25.0}, 150);
//  TestExecutor te(150, 1.0, TestExecutor::SMDP, SMDPFunctions::LINEARIZED_COST, {0.25, -0.25, -0.25, -0.125}, 150);
//  TestExecutor te(150, 1.0, TestExecutor::MCTS_CONSTRAINED, SMDPFunctions::LINEARIZED_COST, {1, 75, 30}, 60);
  TestExecutor te(150, 1.0, TestExecutor::MCTS_SCALARIZED, SMDPFunctions::LINEARIZED_COST,
      {0.25, -0.25, -0.25, -0.125}, 30);

//  //This is a temporary return to test the LP solver in isolation
//  return EXIT_SUCCESS;

  ros::Rate loop_rate(300);
  while (ros::ok())
  {
    ros::spinOnce();
//    if (te.run(0.0333333333333))
    if (te.run(0.01))
    {
      break;
    }
    loop_rate.sleep();
  }

  te.reportResults();

  return EXIT_SUCCESS;
}
