#include "waypoint_planner/SMDPFunctions.h"

using std::vector;

const uint8_t SMDPFunctions::REWARD = 0;
const uint8_t SMDPFunctions::COLLISION = 1;
const uint8_t SMDPFunctions::INTRUSION = 2;
const uint8_t SMDPFunctions::POWER = 3;
const uint8_t SMDPFunctions::LINEARIZED_COST = 4;

double SMDPFunctions::reward(State s, Action a, uint8_t mode, vector<double> weights)
{
  geometry_msgs::Vector3 default_human_dims;
  default_human_dims.x = 0.5;
  default_human_dims.y = 0.4;
  default_human_dims.z = 1.4;

  vector<geometry_msgs::Point> s_primes;
  vector<double> transition_probabilities;
  transitionModel(s.robotPose(), a, s_primes, transition_probabilities);

  double r = 0;

  for (size_t i = 0; i < s_primes.size(); i ++)
  {
    vector<double> durations;
    vector<double> probabilities;

    a.duration(s.robotPose(), s_primes[i], durations, probabilities);

    double r2 = 0;
    for (size_t j = 0; j < durations.size(); j++)
    {
      if (a.actionType() == Action::OBSERVE)
      {
        switch (mode)
        {
          case REWARD:
            r2 += probabilities[j]
                * RewardsAndCosts::reward_recognition(s.humanPose(), default_human_dims, s.robotPose()) * durations[j];
          break;
          case COLLISION:
            r2 += probabilities[j] * RewardsAndCosts::cost_collision(s.humanPose(), default_human_dims, s.robotPose())
                  * durations[j];
          break;
          case INTRUSION:
            r2 += probabilities[j] * RewardsAndCosts::cost_intrusion(s.humanPose(), s.robotPose()) * durations[j];
          break;
          case POWER:
            r2 += probabilities[j] * 0 * durations[j];
          break;
          case LINEARIZED_COST:
            r2 += probabilities[j] * linearizedCost(s.humanPose(), default_human_dims, s.robotPose(), weights)
                * durations[j];
          break;
        }
      }
      else
      {
        r2 += probabilities[j] * 0 * durations[j];  //TODO: reward/cost while in transit? (currently set to 0)
      }
    }

    r += transition_probabilities[i]*r2;
  }

  return r;
}

void SMDPFunctions::transitionModel(geometry_msgs::Point s, Action a, vector<geometry_msgs::Point> &s_primes,
    vector<double> &probabilities)
{
  geometry_msgs::Point s_prime;
  if (a.actionType() == Action::OBSERVE)
  {
    s_prime.x = s.x;
    s_prime.y = s.y;
    s_prime.z = s.z;
  }
  else
  {
    s_prime.x = a.actionGoal().x;
    s_prime.y = a.actionGoal().y;
    s_prime.z = a.actionGoal().z;
  }

  s_primes.push_back(s_prime);
  probabilities.push_back(1.0);
}

double SMDPFunctions::linearizedCost(geometry_msgs::Pose h, geometry_msgs::Vector3 human_dims, geometry_msgs::Point r,
    vector<double> weights)
{
  return weights[0]*RewardsAndCosts::reward_recognition(h, human_dims, r)
         - weights[1]*RewardsAndCosts::cost_collision(h, human_dims, r)
         - weights[2]*RewardsAndCosts::cost_intrusion(h, r);
}