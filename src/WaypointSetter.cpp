#include "waypoint_planner/WaypointSetter.h"

using namespace std;

WaypointSetter::WaypointSetter() :
    pnh("~")
{
  im_server.reset(new interactive_markers::InteractiveMarkerServer("waypoint_setter", "waypoint_setter", false));
  ros::Duration(0.1).sleep();
  im_server->applyChanges();

  // human pose marker
  visualization_msgs::Marker waypoint_marker;
  waypoint_marker.header.frame_id = "world";
  waypoint_marker.pose.orientation.w = 1.0;
  waypoint_marker.action = visualization_msgs::Marker::ADD;
  waypoint_marker.ns = "waypoint";
  waypoint_marker.id = 0;
  waypoint_marker.type = visualization_msgs::Marker::CUBE;
  waypoint_marker.scale.x = 0.2;
  waypoint_marker.scale.y = 0.2;
  waypoint_marker.scale.z = 0.2;
  waypoint_marker.color.r = 1.0;
  waypoint_marker.color.g = 1.0;
  waypoint_marker.color.b = 0.0;
  waypoint_marker.color.a = 1.0;

  visualization_msgs::InteractiveMarker waypoint_im;
  waypoint_im.header.frame_id = "world";
  waypoint_im.pose.orientation.w = 1.0;
  waypoint_im.scale = 0.25;

  visualization_msgs::InteractiveMarkerControl waypoint_center;
  waypoint_center.name = "waypoint_center";
  waypoint_center.interaction_mode = visualization_msgs::InteractiveMarkerControl::NONE;
  waypoint_center.always_visible = true;
  waypoint_center.markers.push_back(waypoint_marker);

  waypoint_im.controls.push_back(waypoint_center);

  visualization_msgs::InteractiveMarkerControl waypoint_control;
  waypoint_control.name = "waypoint_control";
  waypoint_control.orientation.w = 1;
  waypoint_control.orientation.x = 1;
  waypoint_control.orientation.y = 0;
  waypoint_control.orientation.z = 0;
  waypoint_control.name = "rotate_x";
  waypoint_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  waypoint_im.controls.push_back(waypoint_control);
  waypoint_control.name = "move_x";
  waypoint_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  waypoint_im.controls.push_back(waypoint_control);

  waypoint_control.orientation.w = 1;
  waypoint_control.orientation.x = 0;
  waypoint_control.orientation.y = 1;
  waypoint_control.orientation.z = 0;
  waypoint_control.name = "rotate_y";
  waypoint_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  waypoint_im.controls.push_back(waypoint_control);
  waypoint_control.name = "move_y";
  waypoint_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  waypoint_im.controls.push_back(waypoint_control);

  waypoint_control.orientation.w = 1;
  waypoint_control.orientation.x = 0;
  waypoint_control.orientation.y = 0;
  waypoint_control.orientation.z = 1;
  waypoint_control.name = "rotate_z";
  waypoint_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  waypoint_im.controls.push_back(waypoint_control);
  waypoint_control.name = "move_z";
  waypoint_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  waypoint_im.controls.push_back(waypoint_control);

  waypoint_im.controls.push_back(waypoint_control);

  im_server->insert(waypoint_im);
  im_server->setCallback(waypoint_im.name, boost::bind(&WaypointSetter::pose_updated, this, _1));

  im_server->applyChanges();
}

void WaypointSetter::pose_updated(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  if (feedback->event_type != visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE)
    return;

  cout << "------------------------------------\nMarker pose: " << feedback->pose.position.x << ", " <<
    feedback->pose.position.y << ", " << feedback->pose.position.z << std::endl;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "waypoint_setter");
  WaypointSetter ws;

  ros::spin();

  return EXIT_SUCCESS;
}
