#include "waypoint_planner/TrajectorySetter.h"

using namespace std;

TrajectorySetter::TrajectorySetter() :
    pnh("~")
{
  human_pose.header.frame_id = "world";
  human_pose.pose.orientation.w = 1.0;

  human_dims.x = 0.5;
  human_dims.y = 0.4;
  human_dims.z = 1.4;

  yaml << YAML::BeginSeq;

  publishTFs();
  ros::spinOnce();

  im_server.reset(new interactive_markers::InteractiveMarkerServer("trajectory_setter", "trajectory_setter", false));
  ros::Duration(0.1).sleep();
  im_server->applyChanges();

  initializeMarkers();
}

void TrajectorySetter::initializeMarkers()
{
  // human pose marker
  visualization_msgs::Marker pose_marker;
  pose_marker.header.frame_id = human_pose.header.frame_id;
  pose_marker.pose = human_pose.pose;
  pose_marker.action = visualization_msgs::Marker::ADD;
  pose_marker.ns = "human_pose";
  pose_marker.id = 0;
  pose_marker.type = visualization_msgs::Marker::SPHERE;
  pose_marker.scale.x = 0.1;
  pose_marker.scale.y = 0.1;
  pose_marker.scale.z = 0.1;
  pose_marker.color.r = 1.0;
  pose_marker.color.g = 1.0;
  pose_marker.color.b = 1.0;
  pose_marker.color.a = 1.0;

  visualization_msgs::InteractiveMarker pose_im;
  pose_im.header.frame_id = human_pose.header.frame_id;
  pose_im.pose = human_pose.pose;
  pose_im.scale = 1.25;

  visualization_msgs::InteractiveMarkerControl pose_control;
  pose_control.name = "human_pose_control";
  pose_control.orientation.w = 1;
  pose_control.orientation.x = 1;
  pose_control.orientation.y = 0;
  pose_control.orientation.z = 0;
  pose_control.name = "rotate_x";
  pose_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  pose_im.controls.push_back(pose_control);
  pose_control.name = "move_x";
  pose_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  pose_im.controls.push_back(pose_control);

  pose_control.orientation.w = 1;
  pose_control.orientation.x = 0;
  pose_control.orientation.y = 1;
  pose_control.orientation.z = 0;
  pose_control.name = "rotate_y";
  pose_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  pose_im.controls.push_back(pose_control);
  pose_control.name = "move_y";
  pose_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  pose_im.controls.push_back(pose_control);

  pose_control.orientation.w = 1;
  pose_control.orientation.x = 0;
  pose_control.orientation.y = 0;
  pose_control.orientation.z = 1;
  pose_control.name = "rotate_z";
  pose_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  pose_im.controls.push_back(pose_control);
  pose_control.name = "move_z";
  pose_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  pose_im.controls.push_back(pose_control);

  pose_im.controls.push_back(pose_control);

  im_server->insert(pose_im);
  im_server->setCallback(pose_im.name, boost::bind(&TrajectorySetter::poseUpdated, this, _1));

  // human marker
  visualization_msgs::Marker human_marker = EnvironmentSetup::initializeHumanMarker(human_dims);

  visualization_msgs::InteractiveMarkerControl human_control;
  human_control.name = "human_control";
  human_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
  human_control.always_visible = true;
  human_control.markers.push_back(human_marker);

  visualization_msgs::InteractiveMarker human_im;
  human_im.header.frame_id = "human";
  human_im.pose.orientation.w = 1.0;
  human_im.controls.push_back(human_control);
  human_im.name = human_control.name;

  im_server->insert(human_im);
  im_server->setCallback(human_im.name, boost::bind(&TrajectorySetter::markerClicked, this, _1));

  im_server->applyChanges();
}

void TrajectorySetter::poseUpdated(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  if (feedback->event_type != visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE)
    return;

  human_pose.pose = feedback->pose;
}

void TrajectorySetter::markerClicked(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  if (feedback->event_type != visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK)
    return;

  //TODO: save YAML entry
//  yaml << YAML::BeginSeq;
  yaml << YAML::BeginMap;
  yaml << YAML::Key << "time" << YAML::Value << "0.0";
  yaml << YAML::Key << "pose" << YAML::Value << YAML::BeginMap
    << YAML::Key << "position" << YAML::Value << YAML::BeginMap
      << YAML::Key << "x" << YAML::Value << human_pose.pose.position.x
      << YAML::Key << "y" << YAML::Value << human_pose.pose.position.y
      << YAML::Key << "z" << YAML::Value << human_pose.pose.position.z
    << YAML::EndMap
    << YAML::Key << "orientation" << YAML::Value << YAML::BeginMap
      << YAML::Key << "w" << YAML::Value << human_pose.pose.orientation.w
      << YAML::Key << "x" << YAML::Value << human_pose.pose.orientation.x
      << YAML::Key << "y" << YAML::Value << human_pose.pose.orientation.y
      << YAML::Key << "z" << YAML::Value << human_pose.pose.orientation.z
    << YAML::EndMap
  << YAML::EndMap;
  yaml << YAML::EndMap;
//  yaml << YAML::EndSeq;

  cout << "\n\n----------------------------------\nYAML contents:" << endl;
  cout << yaml.c_str() << endl;
}

void TrajectorySetter::publishTFs()
{
  geometry_msgs::TransformStamped human_tf;

  human_tf.header.frame_id = human_pose.header.frame_id;
  human_tf.child_frame_id = "human";
  human_tf.transform.translation.x = human_pose.pose.position.x;
  human_tf.transform.translation.y = human_pose.pose.position.y;
  human_tf.transform.translation.z = human_pose.pose.position.z;
  human_tf.transform.rotation = human_pose.pose.orientation;

  tf_broadcaster.sendTransform(human_tf);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "trajectory_setter");
  TrajectorySetter ts;

  ros::Rate loop_rate(30);
  while (ros::ok())
  {
    ros::spinOnce();
    ts.publishTFs();
    loop_rate.sleep();
  }

  return EXIT_SUCCESS;
}
