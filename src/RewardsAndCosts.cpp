#include "waypoint_planner/RewardsAndCosts.h"

using std::max;
using std::min;
using std::vector;

double RewardsAndCosts::cost_collision(geometry_msgs::Pose h, geometry_msgs::Vector3 h_dims, geometry_msgs::Point r)
{
  // convert ROS messages to bullet types
  btTransform t_h(btQuaternion(h.orientation.x, h.orientation.y, h.orientation.z, h.orientation.w),
                  btVector3(h.position.x, h.position.y, h.position.z));
  btVector3 r_point(r.x, r.y, r.z);
  btVector3 half_dims(h_dims.x/2.0, h_dims.y/2.0, h_dims.z/2.0);

  // transform robot point into human coordinate frame
  btVector3 r_h = t_h.inverse()*r_point;

  // calculate nearest point on human bounding box
  btVector3 closest_point(r_h);
  closest_point.setMax(-half_dims);
  closest_point.setMin(half_dims);
  double dst = (r_h - closest_point).length();

  return exp(-10*dst);
}

double RewardsAndCosts::cost_intrusion(geometry_msgs::Pose h, geometry_msgs::Point r)
{
  // convert ROS messages to bullet types
  btTransform t_h(btQuaternion(h.orientation.x, h.orientation.y, h.orientation.z, h.orientation.w),
                  btVector3(h.position.x, h.position.y, h.position.z));
  btVector3 r_point(r.x, r.y, r.z);
  btVector3 h_point(0, 0, 0.75);

  // get head point in base frame
  btVector3 head = t_h*h_point;

  // intrusiveness cost as an inverse distance from robot to head
  // TODO: lower when perched?
  double dst = (r_point - head).length();

  return exp(-dst);
}

double RewardsAndCosts::reward_recognition(geometry_msgs::Pose h, geometry_msgs::Vector3 h_dims, geometry_msgs::Point r)
{
  vector<int> temp;

  return reward_recognition(h, h_dims, r, temp);
}

double RewardsAndCosts::reward_recognition(geometry_msgs::Pose h, geometry_msgs::Vector3 h_dims,
    geometry_msgs::Point r, std::vector<int> &point_detections)
{
  // convert ROS messages to bullet types
  btTransform t_h(btQuaternion(h.orientation.x, h.orientation.y, h.orientation.z, h.orientation.w),
                  btVector3(h.position.x, h.position.y, h.position.z));
  btVector3 r_point(r.x, r.y, r.z);
  btVector3 half_dims(h_dims.x/2.0, h_dims.y/2.0, h_dims.z/2.0);

  // transform robot point into human coordinate frame
  btVector3 r_h = t_h.inverse()*r_point;

  // check a fixed set of points
  vector<btVector3> keypoints;
  keypoints.clear();

  // change these values to change the number of points checked
  int x_disc = 5;
  int y_disc = 4;
  int z_disc = 10;

  float min_x = -h_dims.x*0.45;
  float max_x = -min_x;
  float min_y = h_dims.y/2.0 + 0.01;
  float max_y = min_y + h_dims.y;
  float min_z = 0;
  float max_z = min_z + h_dims.z/2.0;

  float x_step = (max_x - min_x)/((float)(x_disc - 1));
  float y_step = (max_y - min_y)/((float)(y_disc - 1));
  float z_step = (max_z - min_z)/((float)(z_disc - 1));

  for (int i = 0; i < x_disc; i ++)
  {
    for (int j = 0; j < y_disc; j ++)
    {
      for (int k = 0; k < z_disc; k ++)
      {
        keypoints.push_back(btVector3(min_x + i*x_step, min_y + j*y_step, min_z + k*z_step));
      }
    }
  }

  int visible_points = 0;
  btVector3 r_vc = btVector3(min_x + (max_x - min_x)/2.0, min_y + (max_y - min_y)/2.0, min_z + (max_z - min_z)/2.0) - r_h;
  point_detections.resize(keypoints.size());

  for (unsigned int i = 0; i < keypoints.size(); i ++)
  {
    btVector3 r_vi = keypoints[i] - r_h;

    // FOV check (see assumptions in header file)
    if (fabs(r_vc.angle(r_vi)) <= M_PI/6.0)
    {
      // simple collision with person check (i.e. occlusion)
      if (!collision_check(r_vi.normalized(), r_h, half_dims, r_vi.length()))
      {
        visible_points ++;
        point_detections[i] = 2;
      }
      else
      {
        point_detections[i] = 1;
      }
    }
    else
    {
      point_detections[i] = 0;
    }
  }

  double reward = ((double)visible_points)/keypoints.size();

  // scale reward to account for distance
  return reward/(0.5*max(2.0, (double)(r_vc.length())));
}

bool RewardsAndCosts::collision_check(btVector3 unit_vec, btVector3 origin, btVector3 half_dims, float dst)
{

  btVector3 vec_frac(1.0f/unit_vec.x(), 1.0f/unit_vec.y(), 1.0f/unit_vec.z());
  btVector3 t1 = (-half_dims - origin)*vec_frac;
  btVector3 t2 = (half_dims - origin)*vec_frac;
  float tmin = max(max(min(t1.x(), t2.x()), min(t1.y(), t2.y())), min(t1.z(), t2.z()));
  float tmax = min(min(max(t1.x(), t2.x()), max(t1.y(), t2.y())), max(t1.z(), t2.z()));

  // intersection is behind
  if (tmax < 0)
  {
    return false;
  }

  // no intersection
  if (tmin > tmax)
  {
    return false;
  }

  // check if intersection occurs before or after point to be viewed
  return (tmin < dst);
}
