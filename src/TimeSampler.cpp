#include "waypoint_planner/TimeSampler.h"

using std::cout;
using std::endl;
using std::vector;
using std::normal_distribution;
using std::max;
using std::setprecision;
using std::fixed;

TimeSampler::TimeSampler() :
    generator(std::chrono::system_clock::now().time_since_epoch().count())
{

}

void TimeSampler::sampleExperimentTimes()
{
  vector<double> ts;
  ts.push_back(generateValue(40, 10));
  ts.push_back(*(ts.end() - 1) + generateValue(3, 1));
  ts.push_back(*(ts.end() - 1) + generateValue(5, 2));
  ts.push_back(*(ts.end() - 1) + generateValue(70, 10));
  ts.push_back(*(ts.end() - 1) + generateValue(2, 1));
  ts.push_back(*(ts.end() - 1) + generateValue(5, 2));
  ts.push_back(*(ts.end() - 1) + generateValue(5, 1));
  ts.push_back(*(ts.end() - 1) + generateValue(50, 10));
  experiment_ts.push_back(ts);
}

void TimeSampler::sampleInspectionTimes()
{
  vector<double> ts;
  ts.push_back(generateValue(15, 4, 3));
  ts.push_back(*(ts.end() - 1) + generateValue(15, 4, 3));
  ts.push_back(*(ts.end() - 1) + generateValue(15, 4, 3));
  ts.push_back(*(ts.end() - 1) + generateValue(3, 1));
  ts.push_back(*(ts.end() - 1) + generateValue(10, 3, 3));
  ts.push_back(*(ts.end() - 1) + generateValue(10, 3, 3));
  ts.push_back(*(ts.end() - 1) + generateValue(10, 3, 3));
  ts.push_back(*(ts.end() - 1) + generateValue(10, 3, 3));
  ts.push_back(*(ts.end() - 1) + generateValue(10, 3, 3));
  ts.push_back(*(ts.end() - 1) + generateValue(10, 3, 3));
  ts.push_back(*(ts.end() - 1) + generateValue(10, 3, 3));
  ts.push_back(*(ts.end() - 1) + generateValue(3, 1));
  ts.push_back(*(ts.end() - 1) + generateValue(14, 4, 3));
  ts.push_back(*(ts.end() - 1) + generateValue(14, 4, 3));
  ts.push_back(*(ts.end() - 1) + generateValue(15, 4, 3));
  ts.push_back(*(ts.end() - 1) + generateValue(16, 4, 3));
  inspection_ts.push_back(ts);
}

void TimeSampler::samplePickPlaceTimes()
{
  vector<double> ts;
  ts.push_back(generateValue(15, 5, 3));
  ts.push_back(*(ts.end() - 1) + generateValue(5, 2));
  ts.push_back(*(ts.end() - 1) + generateValue(10, 4, 3));
  ts.push_back(*(ts.end() - 1) + generateValue(5, 2));
  ts.push_back(*(ts.end() - 1) + generateValue(10, 5, 3));
  ts.push_back(*(ts.end() - 1) + generateValue(5, 2));
  ts.push_back(*(ts.end() - 1) + generateValue(10, 4, 3));
  ts.push_back(*(ts.end() - 1) + generateValue(5, 2));
  ts.push_back(*(ts.end() - 1) + generateValue(15, 5, 3));
  ts.push_back(*(ts.end() - 1) + generateValue(5, 2));
  ts.push_back(*(ts.end() - 1) + generateValue(10, 4, 3));
  ts.push_back(*(ts.end() - 1) + generateValue(3, 1));
  ts.push_back(*(ts.end() - 1) + generateValue(2, 1));
  ts.push_back(*(ts.end() - 1) + generateValue(10, 5, 3));
  ts.push_back(*(ts.end() - 1) + generateValue(5, 2));
  ts.push_back(*(ts.end() - 1) + generateValue(10, 4, 3));
  ts.push_back(*(ts.end() - 1) + generateValue(5, 2));
  ts.push_back(*(ts.end() - 1) + generateValue(15, 5, 3));
  ts.push_back(*(ts.end() - 1) + generateValue(5, 2));
  ts.push_back(*(ts.end() - 1) + generateValue(10, 4, 3));
  ts.push_back(*(ts.end() - 1) + generateValue(5, 2));
  ts.push_back(*(ts.end() - 1) + generateValue(5, 2));
  ts.push_back(*(ts.end() - 1) + generateValue(10, 5, 3));
  pick_place_ts.push_back(ts);
}

double TimeSampler::generateValue(double mean, double stdev, double min)
{
  normal_distribution<double> dist(mean, stdev);
  return (max(min, dist(generator)));
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "time_sampler");
  TimeSampler ts;

  for (int i = 0; i < 5; i ++)
  {
    ts.sampleExperimentTimes();
    ts.sampleInspectionTimes();
    ts.samplePickPlaceTimes();
  }

  cout << "\n\nExperiment Times:" << endl;
  for (auto const &ts_i : ts.experiment_ts)
  {
    cout << "\n\tStart" << endl;
    for (auto t : ts_i)
    {
      cout << "\t" << setprecision(2) << fixed << t << endl;
    }
  }

  cout << "\n\nInspection Times:" << endl;
  for (auto const &ts_i : ts.inspection_ts)
  {
    cout << "\n\tStart" << endl;
    for (auto t : ts_i)
    {
      cout << "\t" << setprecision(2) << fixed << t << endl;
    }
  }

  cout << "\n\nPick-and-Place Times:" << endl;
  for (auto const &ts_i : ts.pick_place_ts)
  {
    cout << "\n\tStart" << endl;
    for (auto t : ts_i)
    {
      cout << "\t" << setprecision(2) << fixed << t << endl;
    }
  }

  return EXIT_SUCCESS;
}
