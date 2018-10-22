#include "waypoint_planner/RewardsAndCosts.h"

using std::max;
using std::min;
using std::vector;

double RewardsAndCosts::cost_collision(geometry_msgs::PoseStamped h, geometry_msgs::Vector3 h_dims,
geometry_msgs::PointStamped r)
{
  // convert ROS messages to bullet types
  btTransform t_h(btQuaternion(h.pose.orientation.x, h.pose.orientation.y, h.pose.orientation.z, h.pose.orientation.w),
                  btVector3(h.pose.position.x, h.pose.position.y, h.pose.position.z));
  btVector3 r_point(r.point.x, r.point.y, r.point.z);
  btVector3 half_dims(h_dims.x/2.0, h_dims.y/2.0, h_dims.z/2.0);

  // transform robot point into human coordinate frame
  btVector3 r_h = t_h.inverse()*r_point;

  // calculate nearest point on human bounding box
  btVector3 closest_point(r_h);
  closest_point.setMax(-half_dims);
  closest_point.setMin(half_dims);
//  closest_point.setX(min(max(closest_point.x(), -half_dims.x()), half_dims.x()));
//  closest_point.setY(min(max(closest_point.y(), -half_dims.y()), half_dims.y()));
//  closest_point.setZ(min(max(closest_point.z(), -half_dims.z()), half_dims.z()));
  double dst = (r_h - closest_point).length();

  return exp(-dst);
}

double RewardsAndCosts::cost_intrusion(geometry_msgs::PoseStamped h, geometry_msgs::PointStamped r)
{
  // convert ROS messages to bullet types
  btTransform t_h(btQuaternion(h.pose.orientation.x, h.pose.orientation.y, h.pose.orientation.z, h.pose.orientation.w),
                  btVector3(h.pose.position.x, h.pose.position.y, h.pose.position.z));
  btVector3 r_point(r.point.x, r.point.y, r.point.z);
  btVector3 h_point(0, 0, 0.75);

  // get head point in base frame
  btVector3 head = t_h*h_point;

  // intrusiveness cost as an inverse distance from robot to head
  // TODO: lower when perched?
  double dst = (r_point - head).length();

  return exp(-dst);
}

double RewardsAndCosts::reward_recognition(geometry_msgs::PoseStamped h, geometry_msgs::Vector3 h_dims,
    geometry_msgs::PointStamped r)
{
  // convert ROS messages to bullet types
  btTransform t_h(btQuaternion(h.pose.orientation.x, h.pose.orientation.y, h.pose.orientation.z, h.pose.orientation.w),
                  btVector3(h.pose.position.x, h.pose.position.y, h.pose.position.z));
  btVector3 r_point(r.point.x, r.point.y, r.point.z);
  btVector3 half_dims(h_dims.x/2.0, h_dims.y/2.0, h_dims.z/2.0);

  // transform robot point into human coordinate frame
  btVector3 r_h = t_h.inverse()*r_point;

  // check a fixed set of points
  vector<btVector3> keypoints;
  float v_x_increment = h_dims.x*0.45;
  float v_y_increment = h_dims.y/2.0;
  float v_z_increment = h_dims.z/4.0;
  keypoints.push_back(btVector3(h.pose.position.x, h.pose.position.y + v_y_increment + 0.01,
      h.pose.position.z + v_z_increment));  // workspace center
  keypoints.push_back(keypoints[0] + btVector3(v_x_increment, v_y_increment, v_z_increment));
  keypoints.push_back(keypoints[0] + btVector3(v_x_increment, v_y_increment, -v_z_increment));
  keypoints.push_back(keypoints[0] + btVector3(v_x_increment, -v_y_increment, v_z_increment));
  keypoints.push_back(keypoints[0] + btVector3(v_x_increment, -v_y_increment, -v_z_increment));
  keypoints.push_back(keypoints[0] + btVector3(-v_x_increment, v_y_increment, v_z_increment));
  keypoints.push_back(keypoints[0] + btVector3(-v_x_increment, v_y_increment, -v_z_increment));
  keypoints.push_back(keypoints[0] + btVector3(-v_x_increment, -v_y_increment, v_z_increment));
  keypoints.push_back(keypoints[0] + btVector3(-v_x_increment, -v_y_increment, -v_z_increment));
  keypoints.push_back(keypoints[0] + btVector3(v_x_increment, v_y_increment, 0));
  keypoints.push_back(keypoints[0] + btVector3(-v_x_increment, v_y_increment, 0));
  keypoints.push_back(keypoints[0] + btVector3(v_x_increment, -v_y_increment, 0));
  keypoints.push_back(keypoints[0] + btVector3(-v_x_increment, -v_y_increment, 0));

  int visible_points = 0;
  btVector3 r_vc = keypoints[0] - r_h;

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
      }
    }
  }

  double reward = ((double)visible_points)/keypoints.size();

  // TODO: scale reward to account for distance
  return reward;
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
