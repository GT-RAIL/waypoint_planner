#include "waypoint_planner/HumanSimulator.h"

using std::string;

HumanSimulator::HumanSimulator() :
    pnh("~")
{
  // read in waypoints
  string trajectory_filename;
  pnh.param<string>("trajectory_file", trajectory_filename, "pick_place_trajectory.yaml");
  std::cout << "Reading trajectory from " << trajectory_filename << std::endl;
  string trajectory_file_path = ros::package::getPath("waypoint_planner") + "/config/" + trajectory_filename;
  trajectory = EnvironmentSetup::readHumanTrajectory(trajectory_file_path);

  // setup time, human marker, and publisher
  pnh.param<double>("speed_factor", speed_factor, 1.0);
  time = 0;
  human_marker = EnvironmentSetup::initializeHumanMarker();
  human_marker_publisher = pnh.advertise<visualization_msgs::Marker>("human_marker", 1, this);
  time_update_subscriber = pnh.subscribe<std_msgs::Float32>("time_update", 0, &HumanSimulator::timeUpdateCallback, this);
}

void HumanSimulator::publishTFs()
{
  geometry_msgs::Pose human_pose = trajectory.getPose(time);

  geometry_msgs::TransformStamped human_tf;
  human_tf.header.frame_id = "world";
  human_tf.child_frame_id = "human";
  human_tf.transform.translation.x = human_pose.position.x;
  human_tf.transform.translation.y = human_pose.position.y;
  human_tf.transform.translation.z = human_pose.position.z;
  human_tf.transform.rotation = human_pose.orientation;

  tf_broadcaster.sendTransform(human_tf);
  human_marker_publisher.publish(human_marker);
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
//    hs.advanceTime(0.0333333333333);
    loop_rate.sleep();
  }

  return EXIT_SUCCESS;
}
