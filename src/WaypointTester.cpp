#include "waypoint_planner/WaypointTester.h"

using namespace std;

WaypointTester::WaypointTester() :
    pnh("~")
{
  // read in waypoints
  string waypoint_filename;
  pnh.param<string>("waypoint_file", waypoint_filename, "iss_waypoints.csv");
  string waypoint_file_path = ros::package::getPath("waypoint_planner") + "/config/" + waypoint_filename;
  EnvironmentSetup::readWaypoints(waypoint_file_path, waypoints);

  current_waypoint = 0;

  human_pose.header.frame_id = "world";
  human_pose.pose.orientation.w = 1.0;

  human_dims.x = 0.5;
  human_dims.y = 0.4;
  human_dims.z = 1.4;

  publish_tfs();
  ros::spinOnce();

  im_server.reset(new interactive_markers::InteractiveMarkerServer("waypoint_tester", "waypoint_tester", false));
  ros::Duration(0.1).sleep();
  im_server->applyChanges();

  initialize_markers();
}

void WaypointTester::initialize_markers()
{

  // markers for each waypoint
  for (unsigned int i = 0; i < waypoints.size(); i ++)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = waypoints[i].header.frame_id;
    marker.pose.position = waypoints[i].point;
    marker.pose.orientation.w = 1.0;
    marker.action = visualization_msgs::Marker::ADD;
    marker.ns = "waypoint";
    marker.id = i;
    marker.type = visualization_msgs::Marker::SPHERE;
    if (i == 0)
    {
      marker.scale.x = 0.3;
      marker.scale.y = 0.3;
      marker.scale.z = 0.3;
      marker.color.r = 0.0;
    }
    else
    {
      marker.scale.x = 0.1;
      marker.scale.y = 0.1;
      marker.scale.z = 0.1;
      marker.color.r = 1.0;
    }
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;

    visualization_msgs::InteractiveMarkerControl control;
    std::stringstream ss;
    ss << marker.ns << marker.id;
    control.name = ss.str();
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
    if (i == 0)
      control.always_visible = true;
    control.markers.push_back(marker);

    visualization_msgs::InteractiveMarker im;
    im.header.frame_id = waypoints[i].header.frame_id;
    im.pose.orientation.w = 1.0;
    im.controls.push_back(control);
    im.name = control.name;

    im_server->insert(im);
    im_server->setCallback(im.name, boost::bind(&WaypointTester::waypoint_clicked, this, _1));
  }

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
  pose_im.scale = 1;

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
  im_server->setCallback(pose_im.name, boost::bind(&WaypointTester::pose_updated, this, _1));

  // human marker
  visualization_msgs::Marker human_marker = EnvironmentSetup::initializeHumanMarker(human_dims);
//  human_marker.header.frame_id = "human";
//  human_marker.pose.orientation.w = 1.0;
//  human_marker.action = visualization_msgs::Marker::ADD;
//  human_marker.ns = "human_box";
//  human_marker.id = 0;
//  human_marker.type = visualization_msgs::Marker::CUBE;
//  human_marker.scale = human_dims;
//  human_marker.color.r = 1.0;
//  human_marker.color.g = 0.0;
//  human_marker.color.b = 0.0;
//  human_marker.color.a = 1.0;

  visualization_msgs::InteractiveMarkerControl human_control;
  human_control.name = "human_control";
  human_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::NONE;
  human_control.always_visible = true;
  human_control.markers.push_back(human_marker);

  // keypoint markers
//  float v_x_increment = human_dims.x*0.45;
//  float v_y_increment = human_dims.y/2.0;
//  float v_z_increment = human_dims.z/4.0;

  // change these values to change the number of points checked
  int x_disc = 5;
  int y_disc = 4;
  int z_disc = 10;

  float min_x = -human_dims.x*0.45;
  float max_x = -min_x;
  float min_y = human_dims.y/2.0 + 0.01;
  float max_y = min_y + 0.3;
  float min_z = 0;
  float max_z = min_z + human_dims.z/2.0;

  float x_step = (max_x - min_x)/((float)(x_disc - 1));
  float y_step = (max_y - min_y)/((float)(y_disc - 1));
  float z_step = (max_z - min_z)/((float)(z_disc - 1));

  visualization_msgs::Marker keypoint_marker;
  keypoint_marker.header.frame_id = "human";
  keypoint_marker.pose.position.x = min_x;
  keypoint_marker.pose.position.y = min_y;
  keypoint_marker.pose.position.z = min_z;
  keypoint_marker.pose.orientation.w = 1.0;
  keypoint_marker.action = visualization_msgs::Marker::ADD;
  keypoint_marker.ns = "keypoint";
  keypoint_marker.id = 0;
  keypoint_marker.type = visualization_msgs::Marker::SPHERE;
  keypoint_marker.scale.x = 0.025;
  keypoint_marker.scale.y = 0.025;
  keypoint_marker.scale.z = 0.025;
  keypoint_marker.color.r = 0.6;
  keypoint_marker.color.g = 0.6;
  keypoint_marker.color.b = 1.0;
  keypoint_marker.color.a = 1.0;
  human_control.markers.push_back(keypoint_marker);
  int id_counter = 0;

  for (int i = 0; i < x_disc; i ++)
  {
    for (int j = 0; j < y_disc; j ++)
    {
      for (int k = 0; k < z_disc; k ++)
      {
        if (i == 0 && j == 0 && k == 0)
          continue;
        id_counter ++;
        visualization_msgs::Marker key_marker = human_control.markers[1];
        key_marker.id = id_counter;
        key_marker.pose.position.x += i*x_step;
        key_marker.pose.position.y += j*y_step;
        key_marker.pose.position.z += k*z_step;
        human_control.markers.push_back(key_marker);
      }
    }
  }

  visualization_msgs::Marker midpoint_marker;
  midpoint_marker.header.frame_id = "human";
  midpoint_marker.pose.position.x = min_x + (max_x - min_x)/2.0;
  midpoint_marker.pose.position.y = min_y + (max_y - min_y)/2.0;
  midpoint_marker.pose.position.z = min_z + (max_z - min_z)/2.0;
  midpoint_marker.pose.orientation.w = 1.0;
  midpoint_marker.action = visualization_msgs::Marker::ADD;
  midpoint_marker.ns = "keypoint";
  midpoint_marker.id = 0;
  midpoint_marker.type = visualization_msgs::Marker::SPHERE;
  midpoint_marker.scale.x = 0.04;
  midpoint_marker.scale.y = 0.04;
  midpoint_marker.scale.z = 0.04;
  midpoint_marker.color.r = 0.75;
  midpoint_marker.color.g = 0.75;
  midpoint_marker.color.b = 0.0;
  midpoint_marker.color.a = 1.0;
  human_control.markers.push_back(midpoint_marker);

  visualization_msgs::InteractiveMarker human_im;
  human_im.header.frame_id = "human";
  human_im.pose.orientation.w = 1.0;
  human_im.controls.push_back(human_control);
  human_im.name = human_control.name;

  im_server->insert(human_im);

  im_server->applyChanges();
}

void WaypointTester::pose_updated(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  if (feedback->event_type != visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE)
    return;

  human_pose.pose = feedback->pose;

  calculate_costs();
}

void WaypointTester::waypoint_clicked(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  if (feedback->event_type != visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK)
    return;

  // demote current marker
  std::stringstream ss;
  ss << "waypoint" << current_waypoint;
  visualization_msgs::InteractiveMarker current_im;
  im_server->get(ss.str(), current_im);
  current_im.controls[0].markers[0].color.r = 1.0;
  current_im.controls[0].markers[0].scale.x = 0.1;
  current_im.controls[0].markers[0].scale.y = 0.1;
  current_im.controls[0].markers[0].scale.z = 0.1;
  current_im.controls[0].always_visible = false;

  im_server->erase(ss.str());
  im_server->insert(current_im);

  visualization_msgs::InteractiveMarker im;
  im_server->get(feedback->marker_name, im);
  im.controls[0].markers[0].color.r = 0.0;
  im.controls[0].markers[0].scale.x = 0.3;
  im.controls[0].markers[0].scale.y = 0.3;
  im.controls[0].markers[0].scale.z = 0.3;
  im.controls[0].always_visible = true;
  current_waypoint = im.controls[0].markers[0].id;

  im_server->erase(feedback->marker_name);
  im_server->insert(im);

  im_server->applyChanges();

  calculate_costs();
}

void WaypointTester::calculate_costs()
{
  visualization_msgs::InteractiveMarker im;
  im_server->get("human_control", im);
  vector<int> point_detections;
  point_detections.resize(im.controls[0].markers.size() - 1);

  ROS_INFO("Updated costs:");
  ROS_INFO("\tRecognition reward: %f", RewardsAndCosts::reward_recognition(human_pose, human_dims,
      waypoints[current_waypoint], point_detections));
  ROS_INFO("\tCollision cost: %f", RewardsAndCosts::cost_collision(human_pose, human_dims,
      waypoints[current_waypoint]));
  ROS_INFO("\tIntrusion cost: %f", RewardsAndCosts::cost_intrusion(human_pose, waypoints[current_waypoint]));

  for (unsigned int i = 1; i < im.controls[0].markers.size() - 1; i ++)
  {
    if (point_detections[i-1] == 0)
    {
      im.controls[0].markers[i].color.r = 1.0;
      im.controls[0].markers[i].color.g = 0.0;
      im.controls[0].markers[i].color.b = 1.0;
    }
    else if (point_detections[i-1] == 1)
    {
      im.controls[0].markers[i].color.r = 1.0;
      im.controls[0].markers[i].color.g = 0.0;
      im.controls[0].markers[i].color.b = 0.0;
    }
    else
    {
      im.controls[0].markers[i].color.r = 0.0;
      im.controls[0].markers[i].color.g = 1.0;
      im.controls[0].markers[i].color.b = 0.0;
    }
  }

  im_server->erase("human_control");
  im_server->insert(im);
  im_server->applyChanges();
}

void WaypointTester::publish_tfs()
{
  geometry_msgs::TransformStamped human_tf;
  geometry_msgs::TransformStamped robot_tf;

  human_tf.header.frame_id = human_pose.header.frame_id;
  human_tf.child_frame_id = "human";
  human_tf.transform.translation.x = human_pose.pose.position.x;
  human_tf.transform.translation.y = human_pose.pose.position.y;
  human_tf.transform.translation.z = human_pose.pose.position.z;
  human_tf.transform.rotation = human_pose.pose.orientation;

  robot_tf.header.frame_id = waypoints[current_waypoint].header.frame_id;
  robot_tf.child_frame_id = "robot";
  robot_tf.transform.translation.x = waypoints[current_waypoint].point.x;
  robot_tf.transform.translation.y = waypoints[current_waypoint].point.y;
  robot_tf.transform.translation.z = waypoints[current_waypoint].point.z;
  robot_tf.transform.rotation.w = 1.0;

  tf_broadcaster.sendTransform(human_tf);
  tf_broadcaster.sendTransform(robot_tf);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "waypoint_tester");
  WaypointTester wt;

  ros::Rate loop_rate(30);
  while (ros::ok())
  {
    ros::spinOnce();
    wt.publish_tfs();
    loop_rate.sleep();
  }

  return EXIT_SUCCESS;
}
