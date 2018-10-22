#include <gtest/gtest.h>
#include "waypoint_planner/RewardsAndCosts.h"

TEST(cost_functions, collision_outside)
{
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

  float result = RewardsAndCosts::cost_collision(pose, dims, p);

  EXPECT_NEAR(result, .02789, 0.0001);
}

TEST(cost_functions, collision_inside)
{
  geometry_msgs::PointStamped p;
  p.header.frame_id = "base";
  p.point.x = -4;
  p.point.y = 2.7;
  p.point.z = -1;

  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "base";
  pose.pose.position.x = -3;
  pose.pose.position.y = 0;
  pose.pose.position.z = -1.5;
  pose.pose.orientation.x = 0;
  pose.pose.orientation.y = 0;
  pose.pose.orientation.z = 0.7071068;
  pose.pose.orientation.w = -0.7071068;

  geometry_msgs::Vector3 dims;
  dims.x = 6;
  dims.y = 3;
  dims.z = 3;

  float result = RewardsAndCosts::cost_collision(pose, dims, p);

  EXPECT_EQ(result, 1);
}

TEST(cost_functions, intrusion_max)
{
  geometry_msgs::PointStamped p;
  p.header.frame_id = "base";
  p.point.x = 5;
  p.point.y = 2;
  p.point.z = -0.5;

  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "base";
  pose.pose.position.x = 5;
  pose.pose.position.y = 2;
  pose.pose.position.z = -1.25;
  pose.pose.orientation.x = 0;
  pose.pose.orientation.y = 0;
  pose.pose.orientation.z = 0;
  pose.pose.orientation.w = 1;

  float result = RewardsAndCosts::cost_intrusion(pose, p);

  EXPECT_EQ(result, 1);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}