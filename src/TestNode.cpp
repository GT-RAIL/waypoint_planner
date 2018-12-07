#include "waypoint_planner/TestNode.h"

TestNode::TestNode()
{

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_node");
  TestNode tn;

  ros::spin();

  return EXIT_SUCCESS;
}
