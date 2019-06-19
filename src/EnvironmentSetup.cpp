#include "waypoint_planner/EnvironmentSetup.h"

using std::fstream;
using std::string;
using std::vector;

void EnvironmentSetup::readWaypoints(std::string file_path, std::vector<geometry_msgs::Point> &waypoints)
{
  fstream waypoint_file(file_path.c_str());
  string line;
  geometry_msgs::Point point;
  while (getline(waypoint_file, line))
  {
    unsigned long i = line.find(',');
    point.x = atof(line.substr(0, i).c_str());
    line = line.substr(i + 1);
    i = line.find(',');
    point.y = atof(line.substr(0, i).c_str());
    point.z = atof(line.substr(i + 1).c_str());
    waypoints.push_back(point);
  }
}

HumanTrajectory EnvironmentSetup::readHumanTrajectory(std::string file_path, bool interpolate, double step,
    bool randomize_trajectory)
{
  HumanTrajectory trajectory;
  geometry_msgs::Pose pose;
  YAML::Node trajectoryNode = YAML::LoadFile(file_path);
  for (size_t i = 0; i < trajectoryNode.size(); i ++)
  {
    YAML::Node entry = trajectoryNode[i];
    double t = entry["time"].as<double>();
    pose.position.x = entry["pose"]["position"]["x"].as<double>();
    pose.position.y = entry["pose"]["position"]["y"].as<double>();
    pose.position.z = entry["pose"]["position"]["z"].as<double>();
    pose.orientation.w = entry["pose"]["orientation"]["w"].as<double>();
    pose.orientation.x = entry["pose"]["orientation"]["x"].as<double>();
    pose.orientation.y = entry["pose"]["orientation"]["y"].as<double>();
    pose.orientation.z = entry["pose"]["orientation"]["z"].as<double>();

    trajectory.addPose(t, pose);
  }

  trajectory.sortKeys();

  if (randomize_trajectory)
  {
    trajectory.randomizeTrajectory();
    trajectory.sortKeys();
  }

  if (interpolate)
  {
    trajectory.interpolate(step);
  }

  return trajectory;
}

visualization_msgs::Marker EnvironmentSetup::initializeHumanMarker()
{
  geometry_msgs::Vector3 human_dims;
  human_dims.x = 0.5;
  human_dims.y = 0.4;
  human_dims.z = 1.4;
  return initializeHumanMarker(human_dims);
}

visualization_msgs::Marker EnvironmentSetup::initializeHumanMarker(geometry_msgs::Vector3 human_dims)
{
  visualization_msgs::Marker human_marker;
  human_marker.header.frame_id = "human";
  human_marker.pose.orientation.w = 1.0;
  human_marker.action = visualization_msgs::Marker::ADD;
  human_marker.ns = "human_box";
  human_marker.id = 0;
  human_marker.type = visualization_msgs::Marker::CUBE;
  human_marker.scale = human_dims;
  human_marker.color.r = 1.0;
  human_marker.color.g = 0.0;
  human_marker.color.b = 0.0;
  human_marker.color.a = 1.0;

  return human_marker;
}
