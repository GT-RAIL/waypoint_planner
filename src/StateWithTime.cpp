#include <waypoint_planner/StateWithTime.h>

StateWithTime::StateWithTime(size_t state_id, size_t time_index)
{
  this->state_id = state_id;
  this->time_index = time_index;
}

size_t StateWithTime::stateToIndex(StateWithTime state, size_t max_time_steps)
{
  return stateToIndex(state.state_id, state.time_index, max_time_steps);
}

size_t StateWithTime::stateToIndex(size_t state_id, size_t time_index, size_t max_time_steps)
{
  return state_id*max_time_steps + time_index;
}