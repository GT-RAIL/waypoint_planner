#include "waypoint_planner/TestExecutor.h"

using std::vector;

TestExecutor::TestExecutor(double horizon, double step, uint8_t mode, vector<double> waypoints) :
    solver(horizon, step, mode, "iss_trajectory.yaml", "iss_waypoints.csv", waypoints),
    current_action(Action::OBSERVE),
    pnh("~")
{
  srand(time(NULL));

  // TODO: this is a test to try the LP solver only...
  ROS_INFO("*********************** Start Point ***********************");
  LPSolver lp(horizon, step);
  lp.constructModel({20, 75});
  ROS_INFO("*********************** Solve Point ***********************");
  lp.solveModel();
  ROS_INFO("************************ End Point *************************");
  return;

  solver.backwardsInduction();
  ROS_INFO("Policy computed.");
  time_horizon = horizon;
  current_time = 0;
  next_decision = 0;
  time_step = step;

  // TODO: better waypoint initialization
  waypoint.x = 11.39;
  waypoint.y = -10.12;
  waypoint.z = 4.45;

  robot_vis_publisher = pnh.advertise<visualization_msgs::Marker>("test_robot_vis", 1, this);

  human_sim_time_client = n.serviceClient<waypoint_planner::ChangeTime>("human_simulator/change_time");

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

void TestExecutor::run(double sim_step)
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

    current_action = solver.getAction(waypoint, current_time);
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
  }

  // update fake execution time
  current_time += sim_step;

  // send time update to human trajectory visualizer
  robot_vis_publisher.publish(robot_marker);
  waypoint_planner::ChangeTime change_time;
  change_time.request.relative = true;
  change_time.request.adjustment = sim_step;
  human_sim_time_client.call(change_time);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_executor");
  vector<double> weights{0.333333, 0.333333, 0.333333};
  TestExecutor te(155, 1.0, SMDPFunctions::LINEARIZED_COST, weights);

  //TODO: This is a temporary return to test the LP solver in isolation
  return EXIT_SUCCESS;

  ros::Rate loop_rate(30);
  while (ros::ok())
  {
    ros::spinOnce();
    te.run(10*0.0333333333333);
    loop_rate.sleep();
  }

  return EXIT_SUCCESS;
}
