#include "waypoint_planner/TestExecutor.h"

using std::vector;

const uint8_t TestExecutor::SMDP = 0;
const uint8_t TestExecutor::LP_SOLVE = 1;
const uint8_t TestExecutor::LP_LOAD = 2;
const uint8_t TestExecutor::MCTS = 3;

TestExecutor::TestExecutor(double horizon, double step, uint8_t approach, uint8_t mode, vector<double> weights) :
    solver(horizon, step, mode, "iss_trajectory.yaml", "iss_waypoints.csv", weights),    // TODO: parameters here for optional values
    lp_solver(horizon, step, "iss_trajectory.yaml", "iss_waypoints.csv"),    // TODO: parameters here for optional values
    mcts_solver(horizon, step, "iss_trajectory.yaml", "iss_waypoints.csv", {1.0, 75.0}, 180.0,
        static_cast<size_t>(horizon/step), 2.0),  // TODO: parameters here for optional values
    mcts_reward_solver(horizon, step, "iss_trajectory.yaml", "iss_waypoints.csv", {1.0, 75.0}, 180.0,
        static_cast<size_t>(horizon/step), 2.0),
    current_action(Action::OBSERVE),
    pnh("~")
{
  c1_hat = 1.0;
  c2_hat = 75.0;

  srand(time(NULL));

  this->approach = approach;

  std::string trajectory_file_path = ros::package::getPath("waypoint_planner") + "/config/iss_trajectory.yaml";
  trajectory = EnvironmentSetup::readHumanTrajectory(trajectory_file_path);

  default_human_dims.x = 0.5;
  default_human_dims.y = 0.4;
  default_human_dims.z = 1.4;

  if (this->approach == LP_SOLVE)
  {
    lp_solver.constructModel({1, 15});  // constraint thresholds {d1, d2}
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

  // TODO: better waypoint initialization
  waypoint.x = 11.39;
  waypoint.y = -10.12;
  waypoint.z = 4.45;

  r = 0;
  c1 = 0;
  c2 = 0;

  robot_vis_publisher = pnh.advertise<visualization_msgs::Marker>("test_robot_vis", 1, this);
  human_sim_time_publisher = n.advertise<std_msgs::Float32>("human_simulator/time_update", 1, this);

  robot_marker.header.frame_id = "world";
  robot_marker.pose.position = waypoint;
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
      waypoint = current_action.actionGoal();
      robot_marker.pose.position = waypoint;
      robot_marker.color.a = 1.0;
    }

    if (approach == SMDP)
    {
      current_action = solver.getAction(waypoint, current_time);
    }
    else if (approach == LP_SOLVE || approach == LP_LOAD)
    {
      current_action = lp_solver.getAction(waypoint, current_time);
    }
    else if (approach == MCTS)
    {
      current_action = mcts_solver.search(waypoint, current_time);
    }

    geometry_msgs::Point goal;
    if (current_action.actionType() == Action::MOVE)
    {
      ROS_INFO("Starting move action.");
      goal = current_action.actionGoal();
      robot_marker.color.a = 0.5;
    }
    else
    {
      ROS_INFO("Observing.");
      goal = waypoint;
    }

    // determine a (fake) execution time
    vector<double> durations;
    vector<double> probabilities;
    current_action.duration(waypoint, goal, durations, probabilities);
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

    double r0 = RewardsAndCosts::reward_recognition(trajectory.getPose(current_time), default_human_dims, waypoint) *
                duration;
    double c1_0 = RewardsAndCosts::cost_collision(trajectory.getPose(current_time), default_human_dims, waypoint) * duration;
    double c2_0 = RewardsAndCosts::cost_intrusion(trajectory.getPose(current_time), waypoint) * duration;
    if (approach == MCTS)
    {
      // update cost thresholds
      c1_hat -= c1_0;
      c2_hat -= c2_0;
      if (c1_hat < 0.01)
      {
        c1_hat = 0.01;
      }
      if (c2_hat < 0.01)
      {
        c2_hat = 0.01;
      }
      mcts_solver.setConstraints({c1_hat, c2_hat});
      //mcts_solver.updateConstraints(current_action.actionGoal(), next_decision);
    }


    // calculate rewards and costs and add them to the totals
    if (current_action.actionType() == Action::OBSERVE)
    {
      r += r0;
      c1 += c1_0;
      c2 += c2_0;
    }

    std::cout << "Time: " << current_time << "\tReward: " << r << ", C1: " << c1 << ", C2: " << c2 << std::endl;
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
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_executor");
  vector<double> weights{0.333333, 0.333333, 0.333333};
//  TestExecutor te(150, 1.0, TestExecutor::LP_LOAD, SMDPFunctions::LINEARIZED_COST, weights);
//  TestExecutor te(150, 1.0, TestExecutor::SMDP, SMDPFunctions::LINEARIZED_COST, weights);
  TestExecutor te(150, 1.0, TestExecutor::MCTS, SMDPFunctions::LINEARIZED_COST, weights);

//  //This is a temporary return to test the LP solver in isolation
//  return EXIT_SUCCESS;

  ros::Rate loop_rate(100000);
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
