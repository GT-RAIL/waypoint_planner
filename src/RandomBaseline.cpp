#include "waypoint_planner/RandomBaseline.h"

using std::pair;
using std::string;
using std::vector;

RandomBaseline::RandomBaseline(string waypoint_file_name) :
    generator(std::chrono::system_clock::now().time_since_epoch().count()),
    uniform_dist(0.0, 1.0)
{
  loadWaypoints(waypoint_file_name);
  SMDPFunctions::initializeActions(waypoints, actions);
}

void RandomBaseline::loadWaypoints(string file_name)
{
  string waypoint_file_path = ros::package::getPath("waypoint_planner") + "/config/" + file_name;
  EnvironmentSetup::readWaypoints(waypoint_file_path, waypoints);
}

Action RandomBaseline::getAction(PerchState s)
{
  if (uniform_dist(generator) < 0.5)
  {
    return Action(Action::OBSERVE);
  }

  if (s.perched)
  {
    return Action(Action::UNPERCH);
  }

  int selection = floor(uniform_dist(generator)*(waypoints.size() + 1));
  if (selection >= waypoints.size())
  {
    return Action(Action::PERCH);
  }
  return Action(Action::MOVE, waypoints[selection]);
}
