#ifndef WAYPOINT_PLANNER_SMDP_FUNCTIONS_H_
#define WAYPOINT_PLANNER_SMDP_FUNCTIONS_H_

// ROS
#include <tf2_bullet/tf2_bullet.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <waypoint_planner/State.h>
#include <waypoint_planner/Action.h>

#include "waypoint_planner/PerchState.h"
#include "waypoint_planner/RewardsAndCosts.h"

class SMDPFunctions
{
public:
    static const uint8_t REWARD;
    static const uint8_t COLLISION;
    static const uint8_t INTRUSION;
    static const uint8_t POWER;
    static const uint8_t LINEARIZED_COST;

  static double reward(State s, Action a, uint8_t mode=REWARD, std::vector<double> weights={});

  static double reward(std::vector<State> states, Action a, uint8_t mode=REWARD, std::vector<double> weights={});

  static void transitionModel(PerchState s, Action a, std::vector<PerchState> &s_primes,
      std::vector<double> &probabilities);

  static double linearizedCost(geometry_msgs::Pose h, geometry_msgs::Vector3 human_dims, geometry_msgs::Point r,
      bool perched, Action a, std::vector<double> weights);

  static void initializeActions(std::vector<geometry_msgs::Point> waypoints, std::vector<Action> &actions);

  static bool isValidAction(PerchState s, Action a);
};

#endif  // WAYPOINT_PLANNER_SMDP_FUNCTIONS_H_
