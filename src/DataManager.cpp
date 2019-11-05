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
  pnh.param<string>("dir", subfolder, "training");
  pnh.param<string>("waypoint_file", waypoint_filename, "iss_waypoints.csv");
  pnh.param<bool>("classifier_mode", classifier_mode, true);
  file_path = ros::package::getPath("waypoint_planner") + "/data/" + subfolder + "/" + data_filename + ".csv";
  std::cout << "Ready to read from " << file_path << std::endl;

  save_1D_tensor = n.serviceClient<waypoint_planner::SaveTensor1D>("tensor_saver/save_1D_tensor");
  save_3D_tensor = n.serviceClient<waypoint_planner::SaveTensor3D>("tensor_saver/save_3D_tensor");

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
    int csv_line = 0;  // TODO: loop through all csv entries
    while (std::getline(datafile, line))
    {
//      ROS_INFO("start");
      if (csv_line % 1 == 0)
      {
        cout << "At csv line " << std::to_string(csv_line) << "." << endl;
      }
      if (line[0] == '-')
      {
        csv_line ++;
        continue;
      }
//      ROS_INFO("0");
      Action action(Action::OBSERVE);
      vector<double> cost_constraints;
      PerchState state;
      vector<geometry_msgs::Pose> trajectory;

      if (!unpackLine(line, action, cost_constraints, state, trajectory))
      {
        csv_line ++;
        continue;
      }

//      cout << std::to_string(action.actionType()) << endl;
//      cout << action.actionGoal().x << endl;
//      cout << action.actionGoal().y << endl;
//      cout << action.actionGoal().z << endl << endl;
//
//      cout << cost_constraints[0] << ", " << cost_constraints[1] << ", " << cost_constraints[2] << ", " << cost_constraints[3] << endl << endl;
//
//      cout << state.waypoint.x << ", " << state.waypoint.y << ", " << state.waypoint.z << "  (" << std::to_string(state.perched) << ")" << endl << endl;
//
//      cout << trajectory.size() << endl;
//      for (size_t i = 0; i < trajectory.size(); i ++)
//      {
//        cout << trajectory[i] << endl;
//      }

//      ROS_INFO("1");
      // set up service objects to save in a python-readable format
      waypoint_planner::SaveTensor1D data_srv;
      waypoint_planner::SaveTensor3D pos_srv;
      waypoint_planner::SaveTensor3D rot_srv;

      pos_srv.request.tensor.data.resize(Approximator::IMAGE_SIZE);
      for (size_t i = 0; i < Approximator::IMAGE_SIZE; i ++)
      {
        pos_srv.request.tensor.data[i].data.resize(Approximator::IMAGE_SIZE);
        for (size_t j = 0; j < Approximator::IMAGE_SIZE; j ++)
        {
          pos_srv.request.tensor.data[i].data[j].data.resize(Approximator::IMAGE_SIZE);
//          for (size_t k = 0; k < Approximator::IMAGE_SIZE; k ++)
//          {
//            pos_srv.request.tensor.data[i].data[j].data[k] = 0;
//          }
        }
      }
      rot_srv.request.tensor.data.resize(Approximator::IMAGE_SIZE);
      for (size_t i = 0; i < Approximator::IMAGE_SIZE; i ++)
      {
        rot_srv.request.tensor.data[i].data.resize(Approximator::IMAGE_SIZE);
        for (size_t j = 0; j < Approximator::IMAGE_SIZE; j ++)
        {
          rot_srv.request.tensor.data[i].data[j].data.resize(Approximator::IMAGE_SIZE);
//          for (size_t k = 0; k < Approximator::IMAGE_SIZE; k ++)
//          {
//            rot_srv.request.tensor.data[i].data[j].data[k] = 0;
//          }
        }
      }

//      ROS_INFO("2");
      approximator.createInput(cost_constraints, state, trajectory, pos_srv.request.tensor, rot_srv.request.tensor);

//      ROS_INFO("3");
//      Tensor pos_image = torch::zeros(IntArrayRef{static_cast<long>(Approximator::IMAGE_SIZE),
//                                                  static_cast<long>(Approximator::IMAGE_SIZE),
//                                                  static_cast<long>(Approximator::IMAGE_SIZE)});
//      Tensor rot_image = torch::zeros(IntArrayRef{static_cast<long>(Approximator::IMAGE_SIZE),
//                                                   static_cast<long>(Approximator::IMAGE_SIZE),
//                                                   static_cast<long>(Approximator::IMAGE_SIZE)});
//      approximator.createInput(cost_constraints, state, trajectory, pos_image, rot_image);

//      pos_srv.request.tensor.data.resize(Approximator::IMAGE_SIZE);
//      size_t dim0 = pos_srv.request.tensor.data.size();
//      size_t dim1 = pos_srv.request.tensor.data[0].data.size();
//      size_t dim2 = pos_srv.request.tensor.data[0].data[0].data.size();
//      for (size_t i = 0; i < dim0; i ++)
//      {
//        pos_srv.request.tensor.data[i].data.resize(Approximator::IMAGE_SIZE);
//        for (size_t j = 0; j < dim1; j ++)
//        {
//          pos_srv.request.tensor.data[i].data[j].data.resize(Approximator::IMAGE_SIZE);
//          for (size_t k = 0; k < dim2; k ++)
//          {
//            pos_srv.request.tensor.data[i].data[j].data[k] = pos_image[i][j][k].item<float>();
//          }
//        }
//      }
//      dim0 = rot_srv.request.tensor.data.size();
//      dim1 = rot_srv.request.tensor.data[0].data.size();
//      dim2 = rot_srv.request.tensor.data[0].data[0].data.size();
//      rot_srv.request.tensor.data.resize(Approximator::IMAGE_SIZE);
//      for (size_t i = 0; i < dim0; i ++)
//      {
//        rot_srv.request.tensor.data[i].data.resize(Approximator::IMAGE_SIZE);
//        for (size_t j = 0; j < dim1; j ++)
//        {
//          rot_srv.request.tensor.data[i].data[j].data.resize(Approximator::IMAGE_SIZE);
//          for (size_t k = 0; k < dim2; k ++)
//          {
//            rot_srv.request.tensor.data[i].data[j].data[k] = rot_image[i][j][k].item<float>();
//          }
//        }
//      }

      data_srv.request.tensor.data.resize(8);
      data_srv.request.tensor.data[0] = static_cast<float>(cost_constraints[0]);
      data_srv.request.tensor.data[1] = static_cast<float>(cost_constraints[1]);
      data_srv.request.tensor.data[2] = static_cast<float>(cost_constraints[2]);
      data_srv.request.tensor.data[3] = state.waypoint.x;
      data_srv.request.tensor.data[4] = state.waypoint.y;
      data_srv.request.tensor.data[5] = state.waypoint.z;
      data_srv.request.tensor.data[6] = static_cast<float>(state.perched);
      data_srv.request.tensor.data[7] = static_cast<float>(cost_constraints[3]);

//      ROS_INFO("4");
      string output_csv_path =  ros::package::getPath("waypoint_planner") + "/data/" + subfolder + "/" + data_filename
        + "-samples.csv";
      string output_tensor_name = data_filename + "-" + std::to_string(csv_line);
      string pos_image_file = ros::package::getPath("waypoint_planner") + "/data/" + subfolder + "/trajectories/"
        + output_tensor_name + "-pos.pt";
      string rot_image_file = ros::package::getPath("waypoint_planner") + "/data/" + subfolder + "/trajectories/"
        + output_tensor_name + "-rot.pt";
      string state_data_file = ros::package::getPath("waypoint_planner") + "/data/" + subfolder + "/trajectories/"
        + output_tensor_name + "-data.pt";

      pos_srv.request.filename = pos_image_file;
      rot_srv.request.filename = rot_image_file;
      data_srv.request.filename = state_data_file;

//      ROS_INFO("5");
      if (!save_1D_tensor.call(data_srv) || !save_3D_tensor.call(pos_srv) || !save_3D_tensor.call(rot_srv))
      {
        ROS_INFO("Failed to call python tensor saving service!");
        return;
      }

//      ROS_INFO("6");
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

//      ROS_INFO("7");
      // output tensor data for pos and rot images (in a python read-able format)
//      save(pos_image, pos_image_file);
//      save(rot_image, rot_image_file);
//      save(state_data, state_data_file);
      csv_line ++;
//      ROS_INFO("8");
    }

    datafile.close();
  }
}

//// C++ version...
//void DataManager::testReadData()
//{
//  string line;
//  ifstream datafile(file_path);
//  if (datafile.is_open())
//  {
//    int i = 0;  // TODO: loop through all csv entries
//    while (getline(datafile, line))
//    {
//      if (line[0] == '-')
//      {
//        continue;
//      }
//      Action action(Action::OBSERVE);
//      vector<double> cost_constraints;
//      PerchState state;
//      vector<geometry_msgs::Pose> trajectory;
//      unpackLine(line, action, cost_constraints, state, trajectory);
//
//      Tensor pos_image = torch::zeros(IntArrayRef{static_cast<long>(Approximator::IMAGE_SIZE),
//                                                  static_cast<long>(Approximator::IMAGE_SIZE),
//                                                  static_cast<long>(Approximator::IMAGE_SIZE)});
//      Tensor rot_image = torch::zeros(IntArrayRef{static_cast<long>(Approximator::IMAGE_SIZE),
//                                                   static_cast<long>(Approximator::IMAGE_SIZE),
//                                                   static_cast<long>(Approximator::IMAGE_SIZE)});
//      approximator.createInput(cost_constraints, state, trajectory, pos_image, rot_image);
//
//      Tensor state_data = torch::zeros(IntArrayRef{8});
//      state_data[0] = static_cast<float>(cost_constraints[0]);
//      state_data[1] = static_cast<float>(cost_constraints[1]);
//      state_data[2] = static_cast<float>(cost_constraints[2]);
//      state_data[3] = state.waypoint.x;
//      state_data[4] = state.waypoint.y;
//      state_data[5] = state.waypoint.z;
//      state_data[6] = static_cast<float>(state.perched);
//      state_data[7] = static_cast<float>(cost_constraints[3]);
//
//      string output_csv_path =  ros::package::getPath("waypoint_planner") + "/data/" + data_filename + "-samples.csv";
//      string output_tensor_name = data_filename + "-" + std::to_string(i);
//      string pos_image_file = ros::package::getPath("waypoint_planner") + "/data/trajectories/" + output_tensor_name
//          + "-pos.pt";
//      string rot_image_file = ros::package::getPath("waypoint_planner") + "/data/trajectories/" + output_tensor_name
//          + "-rot.pt";
//      string state_data_file = ros::package::getPath("waypoint_planner") + "/data/trajectories/" + output_tensor_name
//                               + "-data.pt";
//
//      // write to processed csv file
//      std::ofstream sample_file;
//      sample_file.open(output_csv_path, std::ios::out | std::ios::app);
//      if (classifier_mode)
//      {
//        sample_file << std::to_string(approximator.actionToLabel(action)) << ",";
//      }
//      else
//      {
//        sample_file << std::to_string(action.actionType()) << "," << action.actionGoal().x << ","
//          << action.actionGoal().y << "," << action.actionGoal().z << ",";
//      }
//      sample_file << output_tensor_name << endl;
//      sample_file.close();
//
//      // output tensor data for pos and rot images (in a python read-able format)
//      save(pos_image, pos_image_file);
//      save(rot_image, rot_image_file);
//      save(state_data, state_data_file);
//      i ++;
//    }
//
//    datafile.close();
//  }
//}

bool DataManager::unpackLine(string line, Action &action, vector<double> &cost_constraints, PerchState &state,
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
  if (cost_constraints[cost_constraints.size() - 1] <= 0)
  {
    return false;
  }
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
  return true;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "data_manager");
  DataManager dm;

  dm.testReadData();

  return EXIT_SUCCESS;
}
