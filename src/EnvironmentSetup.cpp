#include "waypoint_planner/EnvironmentSetup.h"

using std::fstream;
using std::normal_distribution;
using std::string;
using std::uniform_real_distribution;
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

void EnvironmentSetup::sampleHumanTrajectories(const string &file_path, vector<HumanTrajectory> &sampled_trajectories,
    int num_samples, long seed, bool interpolate, double step)
{
  if (seed == 0)
  {
    seed = std::chrono::system_clock::now().time_since_epoch().count();
  }
  std::mt19937 gen(seed);  // random seed

  sampled_trajectories.resize(num_samples);
  vector<geometry_msgs::Pose> poses;
  vector<double> times;
  poses.resize(num_samples);
  times.resize(num_samples);
  for (size_t i = 0; i < times.size(); i ++)
  {
    times[i] = 0.0;
  }
  YAML::Node trajectoryNode = YAML::LoadFile(file_path);
  for (size_t i = 0; i < trajectoryNode.size(); i ++)
  {
    YAML::Node entry = trajectoryNode[i];

    // sample times from normal distributions
    auto t_mean = entry["mean_time"].as<double>();
    auto t_stdev = entry["stdev_time"].as<double>();
    normal_distribution<double> time_dist(t_mean, t_stdev);
    for (size_t j = 0; j < num_samples; j ++)
    {
      times[j] += time_dist(gen);
    }

    // update new poses if one is specified, otherwise use previous poses
    if (entry["pose"])
    {
      double pos_stdev = 0;
      if (entry["pos_var"])
      {
        pos_stdev = entry["pos_var"].as<double>() / sqrt(2.0);
      }
      normal_distribution<double> pos_r_dist(0, pos_stdev);
      uniform_real_distribution<double> pos_theta_dist(0, M_PI);
      uniform_real_distribution<double> pos_phi_dist(0, 2 * M_PI);

      double rot_stdev;
      if (entry["rot_var"])
      {
        rot_stdev = entry["rot_var"].as<double>();
      }
      normal_distribution<double> rot_dist(0, rot_stdev);

      for (size_t j = 0; j < num_samples; j ++)
      {
        // add position noise using spherical coordinates
        double r = fabs(pos_r_dist(gen));
        double theta = pos_theta_dist(gen);
        double phi = pos_phi_dist(gen);
        poses[j].position.x = entry["pose"]["position"]["x"].as<double>() + r*sin(theta)*cos(phi);
        poses[j].position.y = entry["pose"]["position"]["y"].as<double>() + r*sin(theta)*sin(phi);
        poses[j].position.z = entry["pose"]["position"]["z"].as<double>() + r*cos(theta);

        // add orientation noise to each of roll, pitch, and yaw
        btQuaternion q_key(entry["pose"]["orientation"]["x"].as<double>(),
            entry["pose"]["orientation"]["y"].as<double>(), entry["pose"]["orientation"]["z"].as<double>(),
            entry["pose"]["orientation"]["w"].as<double>());
        btQuaternion q_rand;
        q_rand.setEuler(rot_dist(gen), rot_dist(gen), rot_dist(gen));
        q_key = q_key*q_rand;
        poses[j].orientation.w = q_key.w();
        poses[j].orientation.x = q_key.x();
        poses[j].orientation.y = q_key.y();
        poses[j].orientation.z = q_key.z();
      }
    }

    for (size_t j = 0; j < num_samples; j ++)
    {
      sampled_trajectories[j].addPose(times[j], poses[j]);
    }
  }

  if (interpolate)
  {
    for (size_t i = 0; i < num_samples; i ++)
    {
      sampled_trajectories[i].sortKeys();
      sampled_trajectories[i].interpolate(step);
    }
  }
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
