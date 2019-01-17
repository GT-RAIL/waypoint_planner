#include "waypoint_planner/TestExecutor.h"

using std::vector;

const uint8_t TestExecutor::SMDP = 0;
const uint8_t TestExecutor::LP_SOLVE = 1;
const uint8_t TestExecutor::LP_LOAD = 2;

TestExecutor::TestExecutor(double horizon, double step, uint8_t approach, uint8_t mode, vector<double> weights) :
    solver(horizon, step, mode, "iss_trajectory.yaml", "iss_waypoints.csv", weights),
    lp_solver(horizon, step, "iss_trajectory.yaml", "iss_waypoints.csv"),
    current_action(Action::OBSERVE),
    pnh("~")
{
  srand(time(NULL));

  this->approach = approach;

  if (this->approach == LP_SOLVE)
  {
    lp_solver.constructModel({1, 75});  // constraint thresholds {d1, d2}
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

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_executor");
  vector<double> weights{0.333333, 0.333333, 0.333333};
//  TestExecutor te(155, 1.0, TestExecutor::LP_LOAD, SMDPFunctions::LINEARIZED_COST, weights);
  TestExecutor te(150, 1.0, TestExecutor::LP_LOAD, SMDPFunctions::LINEARIZED_COST, weights);

//  //This is a temporary return to test the LP solver in isolation
//  return EXIT_SUCCESS;

  ros::Rate loop_rate(1000);
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

  return EXIT_SUCCESS;
}
