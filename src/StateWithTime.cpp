#include <waypoint_planner/StateWithTime.h>

StateWithTime::StateWithTime(size_t waypoint_id, size_t time_index)
{
  this->waypoint_id = waypoint_id;
  this->time_index = time_index;
}

StateWithTime StateWithTime::indexToState(size_t i, size_t max_time_steps)
{
  return {static_cast<size_t>(floor(static_cast<double>(i)/max_time_steps)), i%max_time_steps};
}

size_t StateWithTime::stateToIndex(StateWithTime state, size_t max_time_steps)
{
  return stateToIndex(state.waypoint_id, state.time_index, max_time_steps);
}

size_t StateWithTime::stateToIndex(size_t waypoint_id, size_t time_index, size_t max_time_steps)
{
  return waypoint_id*max_time_steps + time_index;
}