#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "waypoint_planner/Approximator.h"

using std::fstream;
using std::string;
using std::vector;
using namespace torch;

const size_t Approximator::IMAGE_SIZE = 64;

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

  torch::Tensor tensor = torch::zeros(IntArrayRef{3,3,2});
  tensor[0][1][0] = 2;
  tensor[1][0][0] = 4;
  tensor[0][0][1] = 1;
  std::cout << tensor << std::endl;
  std::cout << tensor.size(0) << ", " << tensor.size(1) << ", " << tensor.size(2) << std::endl;

  img_cloud_pub = pn.advertise< pcl::PointCloud<pcl::PointXYZRGB> >("trajectory_image_vis", 1, true);
}

void Approximator::createInput(vector<double> cost_constraints, PerchState robot_state,
    vector<geometry_msgs::Pose> trajectory)
{
  vector<float> inputs = {static_cast<float>(cost_constraints[0]), static_cast<float>(cost_constraints[1]),
                          static_cast<float>(cost_constraints[2]),
                          static_cast<float>(robot_state.waypoint.x), static_cast<float>(robot_state.waypoint.y),
                          static_cast<float>(robot_state.waypoint.z),
                           static_cast<float>(robot_state.perched)};

  // order: [z][y][x] (depth, row, column)
  Tensor pos_image = torch::zeros(IntArrayRef{64,64,64});
  Tensor rot_image = torch::zeros(IntArrayRef{64,64,64});

  long start_index = std::min(trajectory.size(), static_cast<size_t>(window)) - 1;

  int prev_x = xIndex(trajectory[start_index].position.x);
  int prev_y = yIndex(trajectory[start_index].position.y);
  int prev_z = zIndex(trajectory[start_index].position.z);

  tf2::Quaternion q_tf(trajectory[start_index].orientation.x, trajectory[start_index].orientation.y,
      trajectory[start_index].orientation.z, trajectory[start_index].orientation.w);
  tf2::Matrix3x3 mat_tf(q_tf);
  double roll, pitch, yaw;
  mat_tf.getRPY(roll, pitch, yaw);
  int prev_rol = rotIndex(roll);
  int prev_pit = rotIndex(pitch);
  int prev_yaw = rotIndex(yaw);

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
    }

    // update rotation image with current trajectory point
    tf2::Quaternion q_tf_cur(trajectory[start_index].orientation.x, trajectory[start_index].orientation.y,
                         trajectory[start_index].orientation.z, trajectory[start_index].orientation.w);
    tf2::Matrix3x3 mat_tf_cur(q_tf_cur);
    double roll_cur, pitch_cur, yaw_cur;
    mat_tf_cur.getRPY(roll_cur, pitch_cur, yaw_cur);
    int cur_rol = rotIndex(roll_cur);
    int cur_pit = rotIndex(pitch_cur);
    int cur_yaw = rotIndex(yaw_cur);
//    if ((prev_rol == cur_rol && prev_pit == cur_pit && prev_yaw == cur_yaw) || fabs(cur_rol - prev_rol) > M_PI
//        || fabs(cur_pit - prev_pit) > M_PI || fabs(cur_yaw - prev_yaw) > M_PI)
    if (prev_rol == cur_rol && prev_pit == cur_pit && prev_yaw == cur_yaw)
    {
      // simply overwrite the value to the earlier time intensity
      pos_image[cur_yaw][cur_pit][cur_rol] = static_cast<float>(window - i)/window;
    }
    else
    {
      // interpolate between prev and cur points
      vector< vector<int> > points = interpolatePoints(prev_rol, cur_rol, prev_pit, cur_pit, prev_yaw, cur_yaw);

      for (size_t j = 0; j < points.size(); j ++)
      {
        pos_image[points[j][2]][points[j][1]][points[j][0]] = (static_cast<float>(window - (i + 1))
                                                               + static_cast<float>(j + 1)/points.size())/window;
      }
    }
  }

  ROS_INFO("Constructed position and rotation 3D trajectory image tensors.");
  ROS_INFO("Converting them to point clouds for visualization...");
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr vis_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  vis_cloud->header.frame_id = "world";
  for (size_t k = 0; k < pos_image.size(0); k ++)
  {
    for (size_t j = 0; j < pos_image.size(1); j ++)
    {
      for (size_t i = 0; i < pos_image.size(2); i ++)
      {
        if (pos_image[k][j][i].item<float>() > 0)
        {
          pcl::PointXYZRGB p;
          p.x = i;
          p.y = j;
          p.z = k;
          p.r = static_cast<uint8_t>(pos_image[k][j][i].item<float>()*255);
          p.g = static_cast<uint8_t>(pos_image[k][j][i].item<float>()*255);
          p.b = static_cast<uint8_t>(pos_image[k][j][i].item<float>()*255);
          p.a = 255;
          vis_cloud->points.push_back(p);
        }
        else
        {
          pcl::PointXYZRGB p;
          p.x = i;
          p.y = j;
          p.z = k;
          p.r = 127;
          p.g = 127;
          p.b = 127;
          p.a = 31;
          vis_cloud->points.push_back(p);
        }
      }
    }
  }

  for (size_t k = 0; k < rot_image.size(0); k ++)
  {
    for (size_t j = 0; j < rot_image.size(1); j ++)
    {
      for (size_t i = 0; i < rot_image.size(2); i ++)
      {
        if (rot_image[k][j][i].item<float>() > 0)
        {
          pcl::PointXYZRGB p;
          p.x = i + 1.2;
          p.y = j;
          p.z = k;
          p.r = static_cast<uint8_t>(rot_image[k][j][i].item<float>()*255);
          p.g = static_cast<uint8_t>(rot_image[k][j][i].item<float>()*255);
          p.b = static_cast<uint8_t>(rot_image[k][j][i].item<float>()*255);
          p.a = 255;
          vis_cloud->points.push_back(p);
        }
        else
        {
          pcl::PointXYZRGB p;
          p.x = i + 1.2;
          p.y = j;
          p.z = k;
          p.r = 127;
          p.g = 127;
          p.b = 127;
          p.a = 31;
          vis_cloud->points.push_back(p);
        }
      }
    }
  }
  img_cloud_pub.publish(vis_cloud);
  ROS_INFO("Visualization complete and published to rviz.");
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
  return lround(rot/M_2_PI*(IMAGE_SIZE - 1));
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

//int main(int argc, char **argv) {
//  ros::init(argc, argv, "test_executor");
//
//  torch::Tensor tensor = torch::rand({2, 3});
//  std::cout << tensor << std::endl;
//}
