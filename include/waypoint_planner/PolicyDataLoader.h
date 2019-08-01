#ifndef WAYPOINT_PLANNER_POLICY_DATA_LOADER_H_
#define WAYPOINT_PLANNER_POLICY_DATA_LOADER_H_

#define _GLIBCXX_USE_CXX11_ABI 0

// CPP
#include <fstream>
#include <string>

#include "waypoint_planner/WaypointPolicyDataset.h"

class PolicyDataLoader
{
public:
    PolicyDataLoader();

private:
//  ros::NodeHandle pnh;

  bool classifier_mode;

};

#endif  // WAYPOINT_PLANNER_POLICY_DATA_LOADER_H_
