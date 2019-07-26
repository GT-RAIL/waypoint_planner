#include "waypoint_planner/DataManager.h"

using std::cout;
using std::endl;
using std::ifstream;
using std::string;
using std::vector;
using namespace torch;

DataManager::DataManager() :
    pnh("~"), approximator()
{
  // get data file
  string waypoint_filename;
  pnh.param<string>("data_file", data_filename, "log_experiment");
  pnh.param<string>("waypoint_file", waypoint_filename, "iss_waypoints.csv");
  pnh.param<bool>("classifier_mode", classifier_mode, true);
  std::cout << "Ready to read from data/" << data_filename << ".csv" << std::endl;
  file_path = ros::package::getPath("waypoint_planner") + "/data/" + data_filename + ".csv";

  // set up waypoints for label generation
  if (classifier_mode)
  {
    approximator.readWaypoints(ros::package::getPath("waypoint_planner") + "/config/" + waypoint_filename);
  }
}

void DataManager::testReadData()
{
  string line;
  ifstream datafile(file_path);
  if (datafile.is_open())
  {
    int i = 0;  // TODO: loop through all csv entries
    while (getline(datafile, line))
    {
      if (line[0] == '-')
      {
        continue;
      }
      Action action(Action::OBSERVE);
      vector<double> cost_constraints;
      PerchState state;
      vector<geometry_msgs::Pose> trajectory;
      unpackLine(line, action, cost_constraints, state, trajectory);

      Tensor pos_image = torch::zeros(IntArrayRef{static_cast<long>(Approximator::IMAGE_SIZE),
                                                  static_cast<long>(Approximator::IMAGE_SIZE),
                                                  static_cast<long>(Approximator::IMAGE_SIZE)});
      Tensor rot_image = torch::zeros(IntArrayRef{static_cast<long>(Approximator::IMAGE_SIZE),
                                                   static_cast<long>(Approximator::IMAGE_SIZE),
                                                   static_cast<long>(Approximator::IMAGE_SIZE)});
      approximator.createInput(cost_constraints, state, trajectory, pos_image, rot_image);

      Tensor state_data = torch::zeros(IntArrayRef{8});
      state_data[0] = static_cast<float>(cost_constraints[0]);
      state_data[1] = static_cast<float>(cost_constraints[1]);
      state_data[2] = static_cast<float>(cost_constraints[2]);
      state_data[3] = state.waypoint.x;
      state_data[4] = state.waypoint.y;
      state_data[5] = state.waypoint.z;
      state_data[6] = static_cast<float>(state.perched);
      state_data[7] = static_cast<float>(cost_constraints[3]);

      string output_csv_path =  ros::package::getPath("waypoint_planner") + "/data/" + data_filename + "-samples.csv";
      string output_tensor_name = data_filename + "-" + std::to_string(i);
      string pos_image_file = ros::package::getPath("waypoint_planner") + "/data/trajectories/" + output_tensor_name
          + "-pos.pt";
      string rot_image_file = ros::package::getPath("waypoint_planner") + "/data/trajectories/" + output_tensor_name
          + "-rot.pt";
      string state_data_file = ros::package::getPath("waypoint_planner") + "/data/trajectories/" + output_tensor_name
                               + "-data.pt";

      // write to processed csv file
      std::ofstream sample_file;
      sample_file.open(output_csv_path, std::ios::out | std::ios::app);
      if (classifier_mode)
      {
        sample_file << std::to_string(approximator.actionToLabel(action)) << ",";
      }
      else
      {
        sample_file << std::to_string(action.actionType()) << "," << action.actionGoal().x << ","
          << action.actionGoal().y << "," << action.actionGoal().z << ",";
      }
      sample_file << output_tensor_name << endl;
      sample_file.close();

      // output tensor data for pos and rot images
      save(pos_image, pos_image_file);
      save(rot_image, rot_image_file);
      save(state_data, state_data_file);
      i ++;
    }

    datafile.close();
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

  // Time remaining (store at end of cost_constraints)
  cost_constraints.push_back(atof(line.substr(0, line.find(',')).c_str()));
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

//  cout << "Read line:" << endl;
//  cout << "\tAction: " << std::to_string(action.actionType()) << "(" << action.actionGoal().x << ", "
//      << action.actionGoal().y << ", " << action.actionGoal().z << ")" << endl;
//  cout << "\tCosts: " << cost_constraints[0] << ", " << cost_constraints[1] << ", " << cost_constraints[2] << endl;
//  cout << "\tRobot State: " << state.perched << " - (" << state.waypoint.x << ", " << state.waypoint.y << ", "
//      << state.waypoint.z << ")" << endl;
//  cout << "\tRemaining trajectory size: " << trajectory.size() << endl;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "data_manager");
  DataManager dm;

  dm.testReadData();

  return EXIT_SUCCESS;
}
