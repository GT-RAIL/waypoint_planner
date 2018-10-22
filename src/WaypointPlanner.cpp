#include "waypoint_planner/WaypointPlanner.h"

using namespace std;

WaypointPlanner::WaypointPlanner() :
    pnh("~")
{
//  pnh.param("neighborhood_radius", neighborhoodRadius, 0.02);
  geometry_msgs::PointStamped p;
  p.header.frame_id = "base";
  p.point.x = 2;
  p.point.y = 3;
  p.point.z = 1;

  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "base";
  pose.pose.position.x = 3;
  pose.pose.position.y = -1;
  pose.pose.position.z = 2;
  pose.pose.orientation.x = 0;
  pose.pose.orientation.y = 0;
  pose.pose.orientation.z = -0.7071068;
  pose.pose.orientation.w = 0.7071068;

  geometry_msgs::Vector3 dims;
  dims.x = 1;
  dims.y = 4;
  dims.z = 0.5;

  RewardsAndCosts::cost_collision(pose, dims, p);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "waypoint_planner");
  WaypointPlanner wp;

  ros::spin();

  return EXIT_SUCCESS;
}
