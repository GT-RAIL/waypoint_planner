#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "waypoint_planner/Approximator.h"

using std::cout;
using std::endl;
using std::fstream;
using std::string;
using std::vector;
using namespace torch;

const size_t Approximator::IMAGE_SIZE = 32;

Approximator::Approximator(int window, double min_x, double max_x, double min_y, double max_y, double min_z,
    double max_z) : pn("~")
{
  this->window = window;
  this->min_x = min_x;
  this->max_x = max_x;
  this->min_y = min_y;
  this->max_y = max_y;
  this->min_z = min_z;
  this->max_z = max_z;
  x_range = max_x - min_x;
  y_range = max_y - min_y;
  z_range = max_z - min_z;

  img_cloud_pub = pn.advertise< pcl::PointCloud<pcl::PointXYZRGB> >("trajectory_image_vis", 1, true);
}

void Approximator::readWaypoints(string file_path)
{
  waypoints.clear();
  EnvironmentSetup::readWaypoints(file_path, waypoints);
}

void Approximator::createInput(vector<double> cost_constraints, PerchState robot_state,
    vector<geometry_msgs::Pose> trajectory, Tensor &pos_image, Tensor &rot_image, bool vis)
{
  vector<float> inputs = {static_cast<float>(cost_constraints[0]), static_cast<float>(cost_constraints[1]),
                          static_cast<float>(cost_constraints[2]),
                          static_cast<float>(robot_state.waypoint.x), static_cast<float>(robot_state.waypoint.y),
                          static_cast<float>(robot_state.waypoint.z),
                           static_cast<float>(robot_state.perched), static_cast<float>(cost_constraints[3])};

  // tensor order: [z][y][x] (depth, row, column)

  long start_index = std::min(trajectory.size(), static_cast<size_t>(window)) - 1;

  int prev_x = xIndex(trajectory[start_index].position.x);
  int prev_y = yIndex(trajectory[start_index].position.y);
  int prev_z = zIndex(trajectory[start_index].position.z);

  tf2::Quaternion q_tf(trajectory[start_index].orientation.x, trajectory[start_index].orientation.y,
      trajectory[start_index].orientation.z, trajectory[start_index].orientation.w);
  tf2::Matrix3x3 mat_tf(q_tf);
  double roll_prev, pitch_prev, yaw_prev;
  mat_tf.getRPY(roll_prev, pitch_prev, yaw_prev);
  int prev_rol = rotIndex(roll_prev + M_PI);
  int prev_pit = rotIndex(pitch_prev + M_PI);
  int prev_yaw = rotIndex(yaw_prev + M_PI);

  pos_image[prev_z][prev_y][prev_x] = static_cast<float>(window - start_index)/window;
  rot_image[prev_yaw][prev_pit][prev_rol] = static_cast<float>(window - start_index)/window;

  for (long i = start_index - 1; i >= 0; i --)
  {
    // update position image with current trajectory point
    int cur_x = xIndex(trajectory[i].position.x);
    int cur_y = yIndex(trajectory[i].position.y);
    int cur_z = zIndex(trajectory[i].position.z);
    if (prev_x == cur_x && prev_y == cur_y && prev_z == cur_z)
    {
      // simply overwrite the value to the earlier time intensity
      pos_image[cur_z][cur_y][cur_x] = static_cast<float>(window - i)/window;
    }
    else
    {
      // interpolate between prev and cur points
      vector< vector<int> > points = interpolatePoints(prev_x, cur_x, prev_y, cur_y, prev_z, cur_z);

      for (size_t j = 0; j < points.size(); j ++)
      {
        pos_image[points[j][2]][points[j][1]][points[j][0]] = (static_cast<float>(window - (i + 1))
            + static_cast<float>(j + 1)/points.size())/window;
      }
      prev_x = cur_x;
      prev_y = cur_y;
      prev_z = cur_z;
    }

    // update rotation image with current trajectory point
    tf2::Quaternion q_tf_cur(trajectory[i].orientation.x, trajectory[i].orientation.y,
                         trajectory[i].orientation.z, trajectory[i].orientation.w);
    tf2::Matrix3x3 mat_tf_cur(q_tf_cur);
    double roll_cur, pitch_cur, yaw_cur;
    mat_tf_cur.getRPY(roll_cur, pitch_cur, yaw_cur);
    int cur_rol = rotIndex(roll_cur + M_PI);
    int cur_pit = rotIndex(pitch_cur + M_PI);
    int cur_yaw = rotIndex(yaw_cur + M_PI);
    if ((prev_rol == cur_rol && prev_pit == cur_pit && prev_yaw == cur_yaw) || fabs(roll_cur - roll_prev) > M_PI
        || fabs(pitch_cur - pitch_prev) > M_PI || fabs(yaw_cur - yaw_prev) > M_PI)
    {
      // simply overwrite the value to the earlier time intensity
      rot_image[cur_yaw][cur_pit][cur_rol] = static_cast<float>(window - i)/window;
    }
    else
    {
      // interpolate between prev and cur points
      vector< vector<int> > points = interpolatePoints(prev_rol, cur_rol, prev_pit, cur_pit, prev_yaw, cur_yaw);

      for (size_t j = 0; j < points.size(); j ++)
      {
        rot_image[points[j][2]][points[j][1]][points[j][0]] = (static_cast<float>(window - (i + 1))
                                                               + static_cast<float>(j + 1)/points.size())/window;
      }
    }
    prev_rol = cur_rol;
    prev_pit = cur_pit;
    prev_yaw = cur_yaw;
    roll_prev = roll_cur;
    pitch_prev = pitch_cur;
    yaw_prev = yaw_cur;
  }

  if (vis)
  {
    ROS_INFO("Constructed position and rotation 3D trajectory image tensors.");
    ROS_INFO("Converting them to point clouds for visualization...");
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr vis_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    vis_cloud->header.frame_id = "world";
    int n = 0;
    for (size_t k = 0; k < pos_image.size(0); k++)
    {
      for (size_t j = 0; j < pos_image.size(1); j++)
      {
        for (size_t i = 0; i < pos_image.size(2); i++)
        {
          if (pos_image[k][j][i].item<float>() > 0)
          {
            n++;
            pcl::PointXYZRGB p;
            p.x = static_cast<float>(i) / IMAGE_SIZE;
            p.y = static_cast<float>(j) / IMAGE_SIZE;
            p.z = static_cast<float>(k) / IMAGE_SIZE;
            p.r = static_cast<uint8_t>(pos_image[k][j][i].item<float>() * 255);
            p.g = static_cast<uint8_t>(pos_image[k][j][i].item<float>() * 255);
            p.b = static_cast<uint8_t>(pos_image[k][j][i].item<float>() * 255);
            p.a = 255;
            vis_cloud->points.push_back(p);
          }
          else
          {
//          pcl::PointXYZRGB p;
//          p.x = static_cast<float>(i)/IMAGE_SIZE;
//          p.y = static_cast<float>(j)/IMAGE_SIZE;
//          p.z = static_cast<float>(k)/IMAGE_SIZE;
//          p.r = 127;
//          p.g = 127;
//          p.b = 127;
//          p.a = 31;
//          vis_cloud->points.push_back(p);
          }
        }
      }
    }

    for (size_t k = 0; k < rot_image.size(0); k++)
    {
      for (size_t j = 0; j < rot_image.size(1); j++)
      {
        for (size_t i = 0; i < rot_image.size(2); i++)
        {
          if (rot_image[k][j][i].item<float>() > 0)
          {
            pcl::PointXYZRGB p;
            p.x = static_cast<float>(i) / IMAGE_SIZE + 1.2;
            p.y = static_cast<float>(j) / IMAGE_SIZE;
            p.z = static_cast<float>(k) / IMAGE_SIZE;
            p.r = static_cast<uint8_t>(rot_image[k][j][i].item<float>() * 255);
            p.g = static_cast<uint8_t>(rot_image[k][j][i].item<float>() * 255);
            p.b = static_cast<uint8_t>(rot_image[k][j][i].item<float>() * 255);
            p.a = 255;
            vis_cloud->points.push_back(p);
          }
          else
          {
//          pcl::PointXYZRGB p;
//          p.x = static_cast<float>(i)/IMAGE_SIZE + 1.2;
//          p.y = static_cast<float>(j)/IMAGE_SIZE;
//          p.z = static_cast<float>(k)/IMAGE_SIZE;
//          p.r = 127;
//          p.g = 127;
//          p.b = 127;
//          p.a = 31;
//          vis_cloud->points.push_back(p);
          }
        }
      }
    }
    img_cloud_pub.publish(vis_cloud);
    ROS_INFO("Visualization complete and published to rviz.");
  }
}

void Approximator::createInput(vector<double> cost_constraints, PerchState robot_state,
    vector<geometry_msgs::Pose> trajectory, waypoint_planner::Tensor3D &pos_image,
    waypoint_planner::Tensor3D &rot_image, bool vis)
{
  vector<float> inputs = {static_cast<float>(cost_constraints[0]), static_cast<float>(cost_constraints[1]),
                          static_cast<float>(cost_constraints[2]),
                          static_cast<float>(robot_state.waypoint.x), static_cast<float>(robot_state.waypoint.y),
                          static_cast<float>(robot_state.waypoint.z),
                          static_cast<float>(robot_state.perched), static_cast<float>(cost_constraints[3])};

  // tensor order: [z][y][x] (depth, row, column)

  long start_index = std::min(trajectory.size(), static_cast<size_t>(window)) - 1;

  int prev_x = xIndex(trajectory[start_index].position.x);
  int prev_y = yIndex(trajectory[start_index].position.y);
  int prev_z = zIndex(trajectory[start_index].position.z);

  tf2::Quaternion q_tf(trajectory[start_index].orientation.x, trajectory[start_index].orientation.y,
                       trajectory[start_index].orientation.z, trajectory[start_index].orientation.w);
  tf2::Matrix3x3 mat_tf(q_tf);
  double roll_prev, pitch_prev, yaw_prev;
  mat_tf.getRPY(roll_prev, pitch_prev, yaw_prev);
  int prev_rol = rotIndex(roll_prev + M_PI);
  int prev_pit = rotIndex(pitch_prev + M_PI);
  int prev_yaw = rotIndex(yaw_prev + M_PI);

//  cout << std::to_string(window) << endl;
//  cout << std::to_string(start_index) << endl;
//  cout << std::to_string(prev_z) << ", " << std::to_string(prev_y) << ", " << std::to_string(prev_x) << endl;
  pos_image.data[prev_z].data[prev_y].data[prev_x] = static_cast<float>(window - start_index)/window;
//  cout << std::to_string(prev_yaw) << ", " << std::to_string(prev_pit) << ", " << std::to_string(prev_rol) << endl;
  rot_image.data[prev_yaw].data[prev_pit].data[prev_rol] = static_cast<float>(window - start_index)/window;

  for (long i = start_index - 1; i >= 0; i --)
  {
    // update position image with current trajectory point
    int cur_x = xIndex(trajectory[i].position.x);
    int cur_y = yIndex(trajectory[i].position.y);
    int cur_z = zIndex(trajectory[i].position.z);
    if (prev_x == cur_x && prev_y == cur_y && prev_z == cur_z)
    {
      // simply overwrite the value to the earlier time intensity
      pos_image.data[cur_z].data[cur_y].data[cur_x] = static_cast<float>(window - i)/window;
    }
    else
    {
      // interpolate between prev and cur points
      vector< vector<int> > points = interpolatePoints(prev_x, cur_x, prev_y, cur_y, prev_z, cur_z);

      for (size_t j = 0; j < points.size(); j ++)
      {
        pos_image.data[points[j][2]].data[points[j][1]].data[points[j][0]] = (static_cast<float>(window - (i + 1))
            + static_cast<float>(j + 1)/points.size())/window;
      }
      prev_x = cur_x;
      prev_y = cur_y;
      prev_z = cur_z;
    }

    // update rotation image with current trajectory point
    tf2::Quaternion q_tf_cur(trajectory[i].orientation.x, trajectory[i].orientation.y,
                             trajectory[i].orientation.z, trajectory[i].orientation.w);
    tf2::Matrix3x3 mat_tf_cur(q_tf_cur);
    double roll_cur, pitch_cur, yaw_cur;
    mat_tf_cur.getRPY(roll_cur, pitch_cur, yaw_cur);
    int cur_rol = rotIndex(roll_cur + M_PI);
    int cur_pit = rotIndex(pitch_cur + M_PI);
    int cur_yaw = rotIndex(yaw_cur + M_PI);
    if ((prev_rol == cur_rol && prev_pit == cur_pit && prev_yaw == cur_yaw) || fabs(roll_cur - roll_prev) > M_PI
        || fabs(pitch_cur - pitch_prev) > M_PI || fabs(yaw_cur - yaw_prev) > M_PI)
    {
      // simply overwrite the value to the earlier time intensity
      rot_image.data[cur_yaw].data[cur_pit].data[cur_rol] = static_cast<float>(window - i)/window;
    }
    else
    {
      // interpolate between prev and cur points
      vector< vector<int> > points = interpolatePoints(prev_rol, cur_rol, prev_pit, cur_pit, prev_yaw, cur_yaw);

      for (size_t j = 0; j < points.size(); j ++)
      {
        rot_image.data[points[j][2]].data[points[j][1]].data[points[j][0]] = (static_cast<float>(window - (i + 1))
            + static_cast<float>(j + 1)/points.size())/window;
      }
    }
    prev_rol = cur_rol;
    prev_pit = cur_pit;
    prev_yaw = cur_yaw;
    roll_prev = roll_cur;
    pitch_prev = pitch_cur;
    yaw_prev = yaw_cur;
  }

  if (vis)
  {
    ROS_INFO("Constructed position and rotation 3D trajectory image tensors.");
    ROS_INFO("Converting them to point clouds for visualization...");
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr vis_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    vis_cloud->header.frame_id = "world";
    int n = 0;
    for (size_t k = 0; k < pos_image.data.size(); k++)
    {
      for (size_t j = 0; j < pos_image.data[k].data.size(); j++)
      {
        for (size_t i = 0; i < pos_image.data[k].data[j].data.size(); i++)
        {
          if (pos_image.data[k].data[j].data[i] > 0)
          {
            n++;
            pcl::PointXYZRGB p;
            p.x = static_cast<float>(i) / IMAGE_SIZE;
            p.y = static_cast<float>(j) / IMAGE_SIZE;
            p.z = static_cast<float>(k) / IMAGE_SIZE;
            p.r = static_cast<uint8_t>(pos_image.data[k].data[j].data[i] * 255);
            p.g = static_cast<uint8_t>(pos_image.data[k].data[j].data[i] * 255);
            p.b = static_cast<uint8_t>(pos_image.data[k].data[j].data[i] * 255);
            p.a = 255;
            vis_cloud->points.push_back(p);
          }
        }
      }
    }

    for (size_t k = 0; k < rot_image.data.size(); k++)
    {
      for (size_t j = 0; j < rot_image.data[k].data.size(); j++)
      {
        for (size_t i = 0; i < rot_image.data[k].data[j].data.size(); i++)
        {
          if (rot_image.data[k].data[j].data[i] > 0)
          {
            pcl::PointXYZRGB p;
            p.x = static_cast<float>(i) / IMAGE_SIZE + 1.2;
            p.y = static_cast<float>(j) / IMAGE_SIZE;
            p.z = static_cast<float>(k) / IMAGE_SIZE;
            p.r = static_cast<uint8_t>(rot_image.data[k].data[j].data[i] * 255);
            p.g = static_cast<uint8_t>(rot_image.data[k].data[j].data[i] * 255);
            p.b = static_cast<uint8_t>(rot_image.data[k].data[j].data[i] * 255);
            p.a = 255;
            vis_cloud->points.push_back(p);
          }
        }
      }
    }
    img_cloud_pub.publish(vis_cloud);
    ROS_INFO("Visualization complete and published to rviz.");
  }
}

int Approximator::xIndex(double x)
{
  return lround((x - min_x)/x_range*(IMAGE_SIZE - 1));
}

int Approximator::yIndex(double y)
{
  return lround((y - min_y)/y_range*(IMAGE_SIZE - 1));
}

int Approximator::zIndex(double z)
{
  return lround((z - min_z)/z_range*(IMAGE_SIZE - 1));
}

int Approximator::rotIndex(double rot)
{
  return lround(rot/(2*M_PI)*(IMAGE_SIZE - 1));
}

std::vector< std::vector<int> > Approximator::interpolatePoints(int x1, int x2, int y1, int y2, int z1, int z2)
{
  vector< vector<int> > points;
  int dx = abs(x2 - x1);
  int dy = abs(y2 - y1);
  int dz = abs(z2 - z1);
  int xs = -1, ys = -1, zs = -1;
  if (x2 > x1)
  {
    xs = 1;
  }
  if (y2 > y1)
  {
    ys = 1;
  }
  if (z2 > z1)
  {
    zs = 1;
  }

  if (dx >= dy && dx >= dz)
  {
    int p1 = 2*dy - dx;
    int p2 = 2*dz - dx;
    while (x1 != x2)
    {
      x1 += xs;
      if (p1 >= 0)
      {
        y1 += ys;
        p1 -= 2*dx;
      }
      if (p2 >= 0)
      {
        z1 += zs;
        p2 -= 2*dx;
      }
      p1 += 2*dy;
      p2 += 2*dz;
      points.push_back({x1, y1, z1});
    }
  }
  else if (dy >= dx and dy >= dz)
  {
    int p1 = 2*dx - dy;
    int p2 = 2*dz - dy;
    while (y1 != y2)
    {
      y1 += ys;
      if (p1 >= 0)
      {
        x1 += xs;
        p1 -= 2 * dy;
      }
      if (p2 >= 0)
      {
        z1 += zs;
        p2 -= 2*dy;
      }
      p1 += 2*dx;
      p2 += 2*dz;
      points.push_back({x1, y1, z1});
    }
  }
  else
  {
    int p1 = 2*dy - dz;
    int p2 = 2*dx - dz;
    while (z1 != z2)
    {
      z1 += zs;
      if (p1 >= 0)
      {
        y1 += ys;
        p1 -= 2*dz;
      }
      if (p2 >= 0)
      {
        x1 += xs;
        p2 -= 2*dz;
      }
      p1 += 2*dy;
      p2 += 2*dx;
      points.push_back({x1, y1, z1});
    }
  }

  return points;
}

int Approximator::actionToLabel(Action action)
{
  if (action.actionType() == Action::OBSERVE)
  {
    return 0;
  }
  else if (action.actionType() == Action::MOVE)
  {
    for (size_t i = 0; i < waypoints.size(); i ++)
    {
      if (action.actionGoal().x == waypoints[i].x && action.actionGoal().y == waypoints[i].y
        && action.actionGoal().z == waypoints[i].z)
      {
        return 1 + i;
      }
    }
    ROS_INFO("Invalid move action goal; cannot convert move action to label!");
    return 0;  // invalid waypoint, return observe instead
  }
  else if (action.actionType() == Action::PERCH)
  {
    return 1 + waypoints.size();
  }
  else if (action.actionType() == Action::UNPERCH)
  {
    return 2 + waypoints.size();
  }

  ROS_INFO("Invalid action type; cannot convert action to label!");
  return 0;  // invalid waypoint, return observe instead
}

void Approximator::labelToAction(int label, Action &action)
{
  if (label == 0)
  {
    action.setActionType(Action::OBSERVE);
  }
  else if (label > waypoints.size())
  {
    if (label == waypoints.size() + 1)
    {
      action.setActionType(Action::PERCH);
    }
    else
    {
      action.setActionType(Action::UNPERCH);
    }
  }
  else
  {
    action.setActionType(Action::MOVE);
    action.setGoal(waypoints[label - 1]);
  }
}

//int main(int argc, char **argv) {
//  ros::init(argc, argv, "test_executor");
//
//  torch::Tensor tensor = torch::rand({2, 3});
//  std::cout << tensor << std::endl;
//}
