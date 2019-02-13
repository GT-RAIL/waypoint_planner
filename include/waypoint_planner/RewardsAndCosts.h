#ifndef WAYPOINT_PLANNER_REWARDS_AND_COSTS_H_
#define WAYPOINT_PLANNER_REWARDS_AND_COSTS_H_

// ROS
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <tf2_bullet/tf2_bullet.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "waypoint_planner/PerchState.h"
#include "waypoint_planner/Action.h"


/*
 * Assumptions:
 *   coordinate frame of person: y is forward, x is right, z is up
 *   FOV of camera performing detection: 60 degrees
 *     (this is either generous, as it doesn't account for orientation, or an underestimate if we are using the
 *     wide-angle NavCam)
*    occlusion check (for quick viewpoint reward approximation) is with person's bounding volume only
 */
class RewardsAndCosts
{
public:
  static double cost_collision(geometry_msgs::Pose h, geometry_msgs::Vector3 h_dims,
      geometry_msgs::Point r);

  static double cost_intrusion(geometry_msgs::Pose h, geometry_msgs::Point r);

  static double cost_power(bool perched, Action a);

  static double reward_recognition(geometry_msgs::Pose h, geometry_msgs::Vector3 h_dims,
      geometry_msgs::Point r);

  static double reward_recognition(geometry_msgs::Pose h, geometry_msgs::Vector3 h_dims,
      geometry_msgs::Point r, std::vector<int> &point_detections);

  static bool collision_check(btVector3 unit_vec, btVector3 origin, btVector3 half_dims, float dst);
};

#endif  // WAYPOINT_PLANNER_REWARDS_AND_COSTS_H_
