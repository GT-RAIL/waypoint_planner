#ifndef WAYPOINT_PLANNER_TIME_SAMPLER_H_
#define WAYPOINT_PLANNER_TIME_SAMPLER_H_

// CPP
#include <algorithm>
#include <chrono>
#include <random>

// ROS
#include <ros/ros.h>

class TimeSampler
{
public:
  TimeSampler();

  void sampleExperimentTimes();

  void sampleInspectionTimes();

  void samplePickPlaceTimes();

  std::vector< std::vector<double> > experiment_ts;
  std::vector< std::vector<double> > inspection_ts;
  std::vector< std::vector<double> > pick_place_ts;

private:
  double generateValue(double mean, double stdev, double min = 1.0);

  std::mt19937 generator;
};

#endif  // WAYPOINT_PLANNER_TIME_SAMPLER_H_
