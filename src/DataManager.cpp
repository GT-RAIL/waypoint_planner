#include "waypoint_planner/DataManager.h"

using std::cout;
using std::endl;
using std::ifstream;
using std::string;
using std::vector;

DataManager::DataManager() :
    pnh("~"), approximator()
{
  // get data file
  string data_filename;
  pnh.param<string>("data_file", data_filename, "log_experiment.txt");
  std::cout << "Ready to read from data/" << data_filename << std::endl;
  file_path = ros::package::getPath("waypoint_planner") + "/data/" + data_filename;
}

void DataManager::testReadData()
{
  string line;
  ifstream datafile(file_path);
  if (datafile.is_open())
  {
    getline(datafile, line);
    Action action(Action::OBSERVE);
    vector<double> cost_constraints;
    PerchState state;
    vector<geometry_msgs::Pose> trajectory;
    unpackLine(line, action, cost_constraints, state, trajectory);
    datafile.close();

    approximator.createInput(cost_constraints, state, trajectory);
  }
}

void DataManager::unpackLine(string line, Action &action, vector<double> &cost_constraints, PerchState &state,
    vector<geometry_msgs::Pose> &trajectory)
{
  // Action (label)
  action.setActionType(atoi(line.substr(0, line.find(',')).c_str()));
  line = line.substr(line.find(',') + 1);

  geometry_msgs::Point goal;
  goal.x = atof(line.substr(0, line.find(',')).c_str());
  line = line.substr(line.find(',') + 1);
  goal.y = atof(line.substr(0, line.find(',')).c_str());
  line = line.substr(line.find(',') + 1);
  goal.z = atof(line.substr(0, line.find(',')).c_str());
  line = line.substr(line.find(',') + 1);
  action.setGoal(goal);

  // Costs
  cost_constraints.push_back(atof(line.substr(0, line.find(',')).c_str()));
  line = line.substr(line.find(',') + 1);
  cost_constraints.push_back(atof(line.substr(0, line.find(',')).c_str()));
  line = line.substr(line.find(',') + 1);
  cost_constraints.push_back(atof(line.substr(0, line.find(',')).c_str()));
  line = line.substr(line.find(',') + 1);

  // Robot State
  state.waypoint.x = atof(line.substr(0, line.find(',')).c_str());
  line = line.substr(line.find(',') + 1);
  state.waypoint.y = atof(line.substr(0, line.find(',')).c_str());
  line = line.substr(line.find(',') + 1);
  state.waypoint.z = atof(line.substr(0, line.find(',')).c_str());
  line = line.substr(line.find(',') + 1);

  state.perched = atoi(line.substr(0, line.find(',')).c_str());
  line = line.substr(line.find(',') + 1);

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
    trajectory.push_back(pose);
  }

  cout << "Read line:" << endl;
  cout << "\tAction: " << std::to_string(action.actionType()) << "(" << action.actionGoal().x << ", "
      << action.actionGoal().y << ", " << action.actionGoal().z << ")" << endl;
  cout << "\tCosts: " << cost_constraints[0] << ", " << cost_constraints[1] << ", " << cost_constraints[2] << endl;
  cout << "\tRobot State: " << state.perched << " - (" << state.waypoint.x << ", " << state.waypoint.y << ", "
      << state.waypoint.z << ")" << endl;
  cout << "\tRemaining trajectory size: " << trajectory.size() << endl;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "data_manager");
  DataManager dm;

  dm.testReadData();

  return EXIT_SUCCESS;
}
