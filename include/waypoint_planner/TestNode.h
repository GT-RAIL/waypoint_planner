#ifndef WAYPOINT_PLANNER_TEST_NODE_H_
#define WAYPOINT_PLANNER_TEST_NODE_H_

// ROS
#include <ros/ros.h>

#include "waypoint_planner/State.h"
#include "waypoint_planner/Action.h"
//#include "waypoint_planner/SMDPSolver.h"

class TestNode
{
public:
    TestNode();

private:
  ros::NodeHandle n, pnh;
};

#endif  // WAYPOINT_PLANNER_TEST_NODE_H_
