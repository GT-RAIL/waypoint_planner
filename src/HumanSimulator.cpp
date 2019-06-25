#include "waypoint_planner/HumanSimulator.h"

using std::string;
using std::vector;

HumanSimulator::HumanSimulator() :
    pnh("~")
{
  // read in waypoints
  string trajectory_filename;
  int num_samples;
  pnh.param<string>("trajectory_file", trajectory_filename, "inspection_times.yaml");
  pnh.param<int>("num_samples", num_samples, 1);
  std::cout << "Creating trajectories from " << trajectory_filename << std::endl;
  string trajectory_file_path = ros::package::getPath("waypoint_planner") + "/config/" + trajectory_filename;
  EnvironmentSetup::sampleHumanTrajectories(trajectory_file_path, trajectories, num_samples);

//  int task_vis;
//  pnh.param<int>("task", task_vis, 1);

  // setup time, human marker, and publisher
  pnh.param<double>("speed_factor", speed_factor, 1.0);
  time = 0;
  for (size_t i = 0; i < trajectories.size(); i ++)
  {
    human_marker = EnvironmentSetup::initializeHumanMarker();
    addHumanMarker("human-" + std::to_string(i), 0.4, 0.8, 1.0, false);
  }

  human_marker_publisher = pnh.advertise<visualization_msgs::Marker>("human_marker", 1, this);
  human_markers_publisher = pnh.advertise<visualization_msgs::MarkerArray>("human_markers", 1, this);
  task_markers_publisher = pnh.advertise<visualization_msgs::MarkerArray>("task_markers", 1, this);
  time_update_subscriber = pnh.subscribe<std_msgs::Float32>("time_update", 0, &HumanSimulator::timeUpdateCallback,
      this);
  trajectory_update_subscriber = pnh.subscribe<waypoint_planner::HumanTrajectoryMsg>("trajectory_update", 0,
      &HumanSimulator::trajectoryUpdateCallback, this);

//  createTaskMarkers(task_vis);
}

void HumanSimulator::trajectoryUpdateCallback(const waypoint_planner::HumanTrajectoryMsg::ConstPtr& msg)
{
  trajectories.clear();
  HumanTrajectory traj;
  traj.fromMsg(*msg);
  trajectories.push_back(traj);
}

void HumanSimulator::addHumanMarker(string frame, double r, double g, double b, bool task_image)
{
  visualization_msgs::Marker head;
  head.header.frame_id = frame;
  head.pose.position.y = -0.05;
  head.pose.position.z = 0.55;
  head.pose.orientation.w = 1.0;
  head.action = visualization_msgs::Marker::ADD;
  head.ns = frame;
  head.id = 0;
  head.type = visualization_msgs::Marker::SPHERE;
  head.scale.x = 0.2;
  head.scale.y = 0.2;
  head.scale.z = 0.2;
  head.color.r = r;
  head.color.g = g;
  head.color.b = b;
  head.color.a = 1.0;

  visualization_msgs::Marker body;
  body.header.frame_id = frame;
  body.pose.position.y = -0.05;
  body.pose.position.z = 0.2;
  body.pose.orientation.w = 1.0;
  body.action = visualization_msgs::Marker::ADD;
  body.ns = frame;
  body.id = 1;
  body.type = visualization_msgs::Marker::CYLINDER;
  body.scale.x = 0.35;
  body.scale.y = 0.2;
  body.scale.z = 0.5;
  body.color.r = r;
  body.color.g = g;
  body.color.b = b;
  body.color.a = 1.0;

  visualization_msgs::Marker leg_l_top;
  leg_l_top.header.frame_id = frame;
  leg_l_top.pose.position.x = -0.085;
  leg_l_top.pose.position.y = 0.05;
  leg_l_top.pose.position.z = -0.2;
  leg_l_top.pose.orientation.x = 0.2588;
  leg_l_top.pose.orientation.y = 0;
  leg_l_top.pose.orientation.z = 0;
  leg_l_top.pose.orientation.w = 0.9659;
  leg_l_top.action = visualization_msgs::Marker::ADD;
  leg_l_top.ns = frame;
  leg_l_top.id = 2;
  leg_l_top.type = visualization_msgs::Marker::CYLINDER;
  leg_l_top.scale.x = 0.16;
  leg_l_top.scale.y = 0.16;
  leg_l_top.scale.z = 0.4;
  leg_l_top.color.r = r;
  leg_l_top.color.g = g;
  leg_l_top.color.b = b;
  leg_l_top.color.a = 1.0;

  visualization_msgs::Marker leg_l_bot;
  leg_l_bot.header.frame_id = frame;
  leg_l_bot.pose.position.x = -0.085;
  leg_l_bot.pose.position.y = 0.025;
  leg_l_bot.pose.position.z = -0.475;
  leg_l_bot.pose.orientation.x = -0.3827;
  leg_l_bot.pose.orientation.y = 0;
  leg_l_bot.pose.orientation.z = 0;
  leg_l_bot.pose.orientation.w = 0.9239;
  leg_l_bot.action = visualization_msgs::Marker::ADD;
  leg_l_bot.ns = frame;
  leg_l_bot.id = 3;
  leg_l_bot.type = visualization_msgs::Marker::CYLINDER;
  leg_l_bot.scale.x = 0.14;
  leg_l_bot.scale.y = 0.14;
  leg_l_bot.scale.z = 0.4;
  leg_l_bot.color.r = r;
  leg_l_bot.color.g = g;
  leg_l_bot.color.b = b;
  leg_l_bot.color.a = 1.0;

  visualization_msgs::Marker leg_r_top;
  leg_r_top.header.frame_id = frame;
  leg_r_top.pose.position.x = 0.085;
  leg_r_top.pose.position.y = 0.05;
  leg_r_top.pose.position.z = -0.2;
  leg_r_top.pose.orientation.x = 0.2588;
  leg_r_top.pose.orientation.y = 0;
  leg_r_top.pose.orientation.z = 0;
  leg_r_top.pose.orientation.w = 0.9659;
  leg_r_top.action = visualization_msgs::Marker::ADD;
  leg_r_top.ns = frame;
  leg_r_top.id = 4;
  leg_r_top.type = visualization_msgs::Marker::CYLINDER;
  leg_r_top.scale.x = 0.16;
  leg_r_top.scale.y = 0.16;
  leg_r_top.scale.z = 0.4;
  leg_r_top.color.r = r;
  leg_r_top.color.g = g;
  leg_r_top.color.b = b;
  leg_r_top.color.a = 1.0;

  visualization_msgs::Marker leg_r_bot;
  leg_r_bot.header.frame_id = frame;
  leg_r_bot.pose.position.x = 0.085;
  leg_r_bot.pose.position.y = 0.025;
  leg_r_bot.pose.position.z = -0.475;
  leg_r_bot.pose.orientation.x = -0.3827;
  leg_r_bot.pose.orientation.y = 0;
  leg_r_bot.pose.orientation.z = 0;
  leg_r_bot.pose.orientation.w = 0.9239;
  leg_r_bot.action = visualization_msgs::Marker::ADD;
  leg_r_bot.ns = frame;
  leg_r_bot.id = 5;
  leg_r_bot.type = visualization_msgs::Marker::CYLINDER;
  leg_r_bot.scale.x = 0.14;
  leg_r_bot.scale.y = 0.14;
  leg_r_bot.scale.z = 0.4;
  leg_r_bot.color.r = r;
  leg_r_bot.color.g = g;
  leg_r_bot.color.b = b;
  leg_r_bot.color.a = 1.0;

  if (task_image)
  {
    task_markers.markers.push_back(head);
    task_markers.markers.push_back(body);
    task_markers.markers.push_back(leg_l_top);
    task_markers.markers.push_back(leg_l_bot);
    task_markers.markers.push_back(leg_r_top);
    task_markers.markers.push_back(leg_r_bot);
  }
  else
  {
    human_markers.markers.push_back(head);
    human_markers.markers.push_back(body);
    human_markers.markers.push_back(leg_l_top);
    human_markers.markers.push_back(leg_l_bot);
    human_markers.markers.push_back(leg_r_top);
    human_markers.markers.push_back(leg_r_bot);
  }
}

void HumanSimulator::createTaskMarkers(int task)
{
  if (task == 0)
  {
    task_tf1.header.frame_id = "world";
    task_tf1.child_frame_id = "task_1";
    task_tf1.transform.translation.x = trajectories[0].getPose(25).position.x;
    task_tf1.transform.translation.y = trajectories[0].getPose(25).position.y;
    task_tf1.transform.translation.z = trajectories[0].getPose(25).position.z;
    task_tf1.transform.rotation = trajectories[0].getPose(15).orientation;

    task_tf2.header.frame_id = "world";
    task_tf2.child_frame_id = "task_2";
    task_tf2.transform.translation.x = trajectories[0].getPose(0).position.x;
    task_tf2.transform.translation.y = trajectories[0].getPose(0).position.y;
    task_tf2.transform.translation.z = trajectories[0].getPose(0).position.z;
    task_tf2.transform.rotation = trajectories[0].getPose(0).orientation;

    task_tf3.header.frame_id = "world";
    task_tf3.child_frame_id = "task_3";
    task_tf3.transform.translation.x = trajectories[0].getPose(0).position.x;
    task_tf3.transform.translation.y = trajectories[0].getPose(0).position.y;
    task_tf3.transform.translation.z = trajectories[0].getPose(0).position.z;
    task_tf3.transform.rotation = trajectories[0].getPose(0).orientation;

    addHumanMarker("task_1", 0.4, 0.8, 1.0);

    visualization_msgs::Marker human_bound = EnvironmentSetup::initializeHumanMarker();
    human_bound.header.frame_id = "task_1";
    human_bound.ns = "bbs";
    human_bound.id = 0;
    human_bound.color.a = 0.4;
    human_bound.scale.y = 0.45;
    task_markers.markers.push_back(human_bound);

    visualization_msgs::Marker roi;
    roi.header.frame_id = "task_1";
    roi.pose.position.x = 0.0;
    roi.pose.position.y = 0.21;
    roi.pose.position.z = 0.35;
    roi.pose.orientation.w = 1.0;
    roi.action = visualization_msgs::Marker::ADD;
    roi.ns = "bbs";
    roi.id = 1;
    roi.type = visualization_msgs::Marker::CUBE;
    roi.scale.x = 0.45;
    roi.scale.y = 0.3;
    roi.scale.z = 0.7;
    roi.color.r = 0.3;
    roi.color.g = 0.6;
    roi.color.b = 1.0;
    roi.color.a = 0.5;
    task_markers.markers.push_back(roi);

    return;
  }

  task_tf1.header.frame_id = "world";
  task_tf1.child_frame_id = "task_1";
  task_tf1.transform.translation.x = trajectories[0].getPose(0).position.x;
  task_tf1.transform.translation.y = trajectories[0].getPose(0).position.y;
  task_tf1.transform.translation.z = trajectories[0].getPose(0).position.z;
  task_tf1.transform.rotation = trajectories[0].getPose(0).orientation;

  addHumanMarker("task_1", 0.4, 0.8, 1.0);

  switch (task)
  {
    case 1:
    {
      double t2 = 45;
      double t3 = 155;

      task_tf2.header.frame_id = "world";
      task_tf2.child_frame_id = "task_2";
      task_tf2.transform.translation.x = trajectories[0].getPose(t2).position.x;
      task_tf2.transform.translation.y = trajectories[0].getPose(t2).position.y;
      task_tf2.transform.translation.z = trajectories[0].getPose(t2).position.z;
      task_tf2.transform.rotation = trajectories[0].getPose(t2).orientation;

      task_tf3.header.frame_id = "world";
      task_tf3.child_frame_id = "task_3";
      task_tf3.transform.translation.x = trajectories[0].getPose(t3).position.x;
      task_tf3.transform.translation.y = trajectories[0].getPose(t3).position.y;
      task_tf3.transform.translation.z = trajectories[0].getPose(t3).position.z;
      task_tf3.transform.rotation = trajectories[0].getPose(t3).orientation;

      addHumanMarker("task_2", 1.0, 1.0, 0.5);
      addHumanMarker("task_3", 1.0, 0.4, 0.4);
    }
      break;
    case 2:
    {
      double t2 = 109;
      double t3 = 180;

      task_tf2.header.frame_id = "world";
      task_tf2.child_frame_id = "task_2";
      task_tf2.transform.translation.x = trajectories[0].getPose(t2).position.x;
      task_tf2.transform.translation.y = trajectories[0].getPose(t2).position.y;
      task_tf2.transform.translation.z = trajectories[0].getPose(t2).position.z;
      task_tf2.transform.rotation = trajectories[0].getPose(t2).orientation;

      task_tf3.header.frame_id = "world";
      task_tf3.child_frame_id = "task_3";
      task_tf3.transform.translation.x = trajectories[0].getPose(t3).position.x;
      task_tf3.transform.translation.y = trajectories[0].getPose(t3).position.y;
      task_tf3.transform.translation.z = trajectories[0].getPose(t3).position.z;
      task_tf3.transform.rotation = trajectories[0].getPose(t3).orientation;

      addHumanMarker("task_2", 1.0, 1.0, 0.5);
      addHumanMarker("task_3", 1.0, 0.4, 0.4);
    }
      break;
    case 3:
    {
      double t2 = 140;
      double t3 = 180;

      task_tf2.header.frame_id = "world";
      task_tf2.child_frame_id = "task_2";
      task_tf2.transform.translation.x = trajectories[0].getPose(t2).position.x;
      task_tf2.transform.translation.y = trajectories[0].getPose(t2).position.y;
      task_tf2.transform.translation.z = trajectories[0].getPose(t2).position.z;
      task_tf2.transform.rotation = trajectories[0].getPose(t2).orientation;

      task_tf3.header.frame_id = "world";
      task_tf3.child_frame_id = "task_3";
      task_tf3.transform.translation.x = trajectories[0].getPose(t3).position.x;
      task_tf3.transform.translation.y = trajectories[0].getPose(t3).position.y;
      task_tf3.transform.translation.z = trajectories[0].getPose(t3).position.z;
      task_tf3.transform.rotation = trajectories[0].getPose(t3).orientation;

      addHumanMarker("task_2", 1.0, 1.0, 0.5);
      addHumanMarker("task_3", 1.0, 0.3, 0.3);
    }
      break;
  }
}

void HumanSimulator::publishTFs()
{
  vector<geometry_msgs::Pose> human_poses;
  human_poses.resize(trajectories.size());
  for (size_t i = 0; i < trajectories.size(); i ++)
  {
    human_poses[i] = trajectories[i].getPose(time);
  }

  for (size_t i = 0; i < human_poses.size(); i ++)
  {
    geometry_msgs::TransformStamped human_tf;
    human_tf.header.frame_id = "world";
    human_tf.child_frame_id = "human-" + std::to_string(i);
    human_tf.transform.translation.x = human_poses[i].position.x;
    human_tf.transform.translation.y = human_poses[i].position.y;
    human_tf.transform.translation.z = human_poses[i].position.z;
    human_tf.transform.rotation = human_poses[i].orientation;
    tf_broadcaster.sendTransform(human_tf);
  }

//  tf_broadcaster.sendTransform(task_tf1);
//  tf_broadcaster.sendTransform(task_tf2);
//  tf_broadcaster.sendTransform(task_tf3);
  human_marker_publisher.publish(human_marker);
  human_markers_publisher.publish(human_markers);
//  task_markers_publisher.publish(task_markers);
}

void HumanSimulator::advanceTime(double timestep)
{
  time += timestep*speed_factor;
}

void HumanSimulator::timeUpdateCallback(const std_msgs::Float32::ConstPtr& msg)
{
  time = msg->data;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "human_simulator");
  HumanSimulator hs;

  ros::Rate loop_rate(30);
  while (ros::ok())
  {
    ros::spinOnce();
    hs.publishTFs();
//    hs.advanceTime(0.333333333333);
    loop_rate.sleep();
  }

  return EXIT_SUCCESS;
}
