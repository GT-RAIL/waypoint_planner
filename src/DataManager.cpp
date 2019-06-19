#include "waypoint_planner/DataManager.h"

using std::cout;
using std::endl;
using std::ifstream;
using std::string;
using std::vector;

DataManager::DataManager() :
    pnh("~")
{
  // get data file
  string data_filename;
  pnh.param<string>("data_file", data_filename, "log_pick_place.txt");
  std::cout << "Ready to read from data/" << data_filename << std::endl;
  file_path = ros::package::getPath("waypoint_planner") + "/data/" + data_filename;
}

void DataManager::readFile()
{
  string line;
  ifstream datafile(file_path);
  if (datafile.is_open())
  {
    getline(datafile, line);
    unpackLine(line);
    datafile.close();
  }
}

void DataManager::unpackLine(string line)
{
  // log data, in the form:
  //  action type, goal x, goal y, goal z, c1 remaining, c2 remaining, c3 remaining, current x, current y, current z,
  //  perched, [remaining human trajectory: x, y, z, qx, qy, qz, qw, ...]

  // Action (label)
  uint8_t a_id = atoi(line.substr(0, line.find(',')).c_str());
  line = line.substr(line.find(',') + 1);

  geometry_msgs::Point goal;
  goal.x = atof(line.substr(0, line.find(',')).c_str());
  line = line.substr(line.find(',') + 1);
  goal.y = atof(line.substr(0, line.find(',')).c_str());
  line = line.substr(line.find(',') + 1);
  goal.z = atof(line.substr(0, line.find(',')).c_str());
  line = line.substr(line.find(',') + 1);

  Action action(a_id, goal);

  // Costs
  vector<double> cost_constraints;
  cost_constraints.push_back(atof(line.substr(0, line.find(',')).c_str()));
  line = line.substr(line.find(',') + 1);
  cost_constraints.push_back(atof(line.substr(0, line.find(',')).c_str()));
  line = line.substr(line.find(',') + 1);
  cost_constraints.push_back(atof(line.substr(0, line.find(',')).c_str()));
  line = line.substr(line.find(',') + 1);

  // Robot State
  geometry_msgs::Point robot_pose;
  robot_pose.x = atof(line.substr(0, line.find(',')).c_str());
  line = line.substr(line.find(',') + 1);
  robot_pose.y = atof(line.substr(0, line.find(',')).c_str());
  line = line.substr(line.find(',') + 1);
  robot_pose.z = atof(line.substr(0, line.find(',')).c_str());
  line = line.substr(line.find(',') + 1);

  bool perched = atoi(line.substr(0, line.find(',')).c_str());
  line = line.substr(line.find(',') + 1);

  PerchState state(robot_pose, perched);

  vector<geometry_msgs::Pose> trajectory;
  while (line.find(',') != string::npos)
  {
    geometry_msgs::Pose pose;
    pose.position.x = atof(line.substr(0, line.find(',')).c_str());
    line = line.substr(line.find(',') + 1);
    pose.position.y = atof(line.substr(0, line.find(',')).c_str());
    line = line.substr(line.find(',') + 1);
    pose.position.z = atof(line.substr(0, line.find(',')).c_str());
    line = line.substr(line.find(',') + 1);
    pose.orientation.x = atof(line.substr(0, line.find(',')).c_str());
    line = line.substr(line.find(',') + 1);
    pose.orientation.y = atof(line.substr(0, line.find(',')).c_str());
    line = line.substr(line.find(',') + 1);
    pose.orientation.z = atof(line.substr(0, line.find(',')).c_str());
    line = line.substr(line.find(',') + 1);
    pose.orientation.w = atof(line.substr(0, line.find(',')).c_str());
    line = line.substr(line.find(',') + 1);
  }

  cout << "Read line:" << endl;
  cout << "\tAction: " << std::to_string(action.actionType()) << "(" << action.actionGoal().x << ", "
      << action.actionGoal().y << ", " << action.actionGoal().z << endl;
  cout << "\tCosts: " << cost_constraints[0] << ", " << cost_constraints[1] << ", " << cost_constraints[2] << endl;
  cout << "\tRobot State: " << state.perched << " - (" << state.waypoint.x << ", " << state.waypoint.y << ", "
      << state.waypoint.z << endl;
  cout << "\tRemaining trajectory size: " << trajectory.size() << endl;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "data_manager");
  DataManager dm;

  dm.readFile();

  return EXIT_SUCCESS;
}
