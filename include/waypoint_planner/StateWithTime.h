#ifndef WAYPOINT_PLANNER_STATE_WITH_TIME_H_
#define WAYPOINT_PLANNER_STATE_WITH_TIME_H_

#include <stdlib.h>
#include <math.h>

class StateWithTime
{
public:
    size_t waypoint_id;
    size_t time_index;

    StateWithTime(size_t waypoint_id, size_t time_index);

    static StateWithTime indexToState(size_t i, size_t max_time_steps);

    static size_t stateToIndex(StateWithTime state, size_t max_time_steps);

    static size_t stateToIndex(size_t waypoint_id, size_t time_index, size_t max_time_steps);
};

#endif  // WAYPOINT_PLANNER_STATE_WITH_TIME_H_
