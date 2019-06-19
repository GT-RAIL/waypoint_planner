#include "waypoint_planner/HumanTrajectory.h"

HumanTrajectory::HumanTrajectory()
{
  srand(time(NULL));
}

void HumanTrajectory::addPose(double time, geometry_msgs::Pose pose)
{
  trajectory.insert(std::pair<double, geometry_msgs::Pose>(time, pose));
  keys.push_back(time);
}

void HumanTrajectory::sortKeys()
{
  std::sort(keys.begin(), keys.end());
}

void HumanTrajectory::randomizeTrajectory(double arc_trans, double arc_rot, double key_trans, double key_rot)
{
  // add a midpoint between key points, perform a randomization on that
  // then perform a different randomization on the original keypoints
  // resulting in a similar overall task with a larger variety of arcs between the original keypoints
  size_t last_key = keys.size() - 1;
  bool no_move = false;
  for (size_t i = 0; i <= last_key; i ++)
  {
    if (i == last_key)
    {
      if (i > 0 && !comparePoses(trajectory[keys[i - 1]], trajectory[keys[i]]))
      {
        // just randomize keypoint
        trajectory[keys[i]].position.x += (2*static_cast<double>(rand())/RAND_MAX - 1)*key_trans;
        trajectory[keys[i]].position.y += (2*static_cast<double>(rand())/RAND_MAX - 1)*key_trans;
        trajectory[keys[i]].position.z += (2*static_cast<double>(rand())/RAND_MAX - 1)*key_trans;

        btQuaternion q_key(trajectory[keys[i]].orientation.x, trajectory[keys[i]].orientation.y,
                           trajectory[keys[i]].orientation.z, trajectory[keys[i]].orientation.w);
        btQuaternion q_rand;
        q_rand.setEuler((2*static_cast<double>(rand())/RAND_MAX - 1)*key_rot,
                        (2*static_cast<double>(rand())/RAND_MAX - 1)*key_rot,
                        (2*static_cast<double>(rand())/RAND_MAX - 1)*key_rot);
        q_key = q_key*q_rand;
        trajectory[i].orientation.w = q_key.w();
        trajectory[i].orientation.x = q_key.x();
        trajectory[i].orientation.y = q_key.y();
        trajectory[i].orientation.z = q_key.z();
      }
      else if (i > 0)
      {
        // set to the previous point
        trajectory[keys[i]].position.x = trajectory[keys[i - 1]].position.x;
        trajectory[keys[i]].position.y = trajectory[keys[i - 1]].position.y;
        trajectory[keys[i]].position.z = trajectory[keys[i - 1]].position.z;
        trajectory[keys[i]].orientation.w = trajectory[keys[i - 1]].orientation.w;
        trajectory[keys[i]].orientation.x = trajectory[keys[i - 1]].orientation.x;
        trajectory[keys[i]].orientation.y = trajectory[keys[i - 1]].orientation.y;
        trajectory[keys[i]].orientation.z = trajectory[keys[i - 1]].orientation.z;
      }
    }
    else if (!comparePoses(trajectory[keys[i]], trajectory[keys[i + 1]]))
    {
      // add a randomized point halfway between the arc
      double t1 = keys[i];
      double t2 = keys[i + 1];
      geometry_msgs::Pose p1 = trajectory[t1];
      geometry_msgs::Pose p2 = trajectory[t2];

      double dt = keys[i + 1] - keys[i];

      double dx = p2.position.x - p1.position.x;
      double dy = p2.position.y - p1.position.y;
      double dz = p2.position.z - p1.position.z;

      btQuaternion q1(p1.orientation.x, p1.orientation.y, p1.orientation.z, p1.orientation.w);
      btQuaternion q2(p2.orientation.x, p2.orientation.y, p2.orientation.z, p2.orientation.w);

      // perform interpolation
      geometry_msgs::Pose addPose;
      double percentage = 0.5;
      double tj = t1 + percentage*dt;

      // interpolate position
      addPose.position.x = p1.position.x + percentage*dx;
      addPose.position.y = p1.position.y + percentage*dy;
      addPose.position.z = p1.position.z + percentage*dz;

      // interpolate (slerp) rotation
      btQuaternion q_slerp = q1.slerp(q2, percentage);
      addPose.orientation.w = q_slerp.w();
      addPose.orientation.x = q_slerp.x();
      addPose.orientation.y = q_slerp.y();
      addPose.orientation.z = q_slerp.z();

      // randomize the intermediate keypoint
      addPose.position.x += (2*static_cast<double>(rand())/RAND_MAX - 1)*arc_trans;
      addPose.position.y += (2*static_cast<double>(rand())/RAND_MAX - 1)*arc_trans;
      addPose.position.z += (2*static_cast<double>(rand())/RAND_MAX - 1)*arc_trans;

      btQuaternion q_add(addPose.orientation.x, addPose.orientation.y, addPose.orientation.z, addPose.orientation.w);
      btQuaternion q_rand;
      q_rand.setEuler((2*static_cast<double>(rand())/RAND_MAX - 1)*arc_rot,
          (2*static_cast<double>(rand())/RAND_MAX - 1)*arc_rot,
          (2*static_cast<double>(rand())/RAND_MAX - 1)*arc_rot);
      q_add = q_add*q_rand;
      addPose.orientation.w = q_add.w();
      addPose.orientation.x = q_add.x();
      addPose.orientation.y = q_add.y();
      addPose.orientation.z = q_add.z();

      keys.push_back(tj);
      trajectory.insert(std::pair<double, geometry_msgs::Pose>(tj, addPose));

      if (no_move)
      {
        // set to the previous point
        trajectory[keys[i]].position.x = trajectory[keys[i - 1]].position.x;
        trajectory[keys[i]].position.y = trajectory[keys[i - 1]].position.y;
        trajectory[keys[i]].position.z = trajectory[keys[i - 1]].position.z;
        trajectory[keys[i]].orientation.w = trajectory[keys[i - 1]].orientation.w;
        trajectory[keys[i]].orientation.x = trajectory[keys[i - 1]].orientation.x;
        trajectory[keys[i]].orientation.y = trajectory[keys[i - 1]].orientation.y;
        trajectory[keys[i]].orientation.z = trajectory[keys[i - 1]].orientation.z;
      }
      else
      {
        // randomize initial keypoint
        trajectory[t1].position.x += (2*static_cast<double>(rand())/RAND_MAX - 1)*key_trans;
        trajectory[t1].position.y += (2*static_cast<double>(rand())/RAND_MAX - 1)*key_trans;
        trajectory[t1].position.z += (2*static_cast<double>(rand())/RAND_MAX - 1)*key_trans;

        btQuaternion q_key(trajectory[t1].orientation.x, trajectory[t1].orientation.y, trajectory[t1].orientation.z,
                           trajectory[t1].orientation.w);
        q_rand.setEuler((2*static_cast<double>(rand())/RAND_MAX - 1)*key_rot,
                        (2*static_cast<double>(rand())/RAND_MAX - 1)*key_rot,
                        (2*static_cast<double>(rand())/RAND_MAX - 1)*key_rot);
        q_key = q_key*q_rand;
        trajectory[t1].orientation.w = q_key.w();
        trajectory[t1].orientation.x = q_key.x();
        trajectory[t1].orientation.y = q_key.y();
        trajectory[t1].orientation.z = q_key.z();
      }
      no_move = false;
    }
    else
    {
      // randomize initial keypoint, set no_move flag so future keypoints preserve standing still
      if (!no_move)
      {
        // randomize initial keypoint
        trajectory[keys[i]].position.x += (2*static_cast<double>(rand())/RAND_MAX - 1)*key_trans;
        trajectory[keys[i]].position.y += (2*static_cast<double>(rand())/RAND_MAX - 1)*key_trans;
        trajectory[keys[i]].position.z += (2*static_cast<double>(rand())/RAND_MAX - 1)*key_trans;

        btQuaternion q_key(trajectory[keys[i]].orientation.x, trajectory[keys[i]].orientation.y,
            trajectory[keys[i]].orientation.z, trajectory[keys[i]].orientation.w);
        btQuaternion q_rand;
        q_rand.setEuler((2*static_cast<double>(rand())/RAND_MAX - 1)*key_rot,
                        (2*static_cast<double>(rand())/RAND_MAX - 1)*key_rot,
                        (2*static_cast<double>(rand())/RAND_MAX - 1)*key_rot);
        q_key = q_key*q_rand;
        trajectory[i].orientation.w = q_key.w();
        trajectory[i].orientation.x = q_key.x();
        trajectory[i].orientation.y = q_key.y();
        trajectory[i].orientation.z = q_key.z();
        no_move = true;
      }
      else
      {
        // set to the previous point
        trajectory[keys[i]].position.x = trajectory[keys[i - 1]].position.x;
        trajectory[keys[i]].position.y = trajectory[keys[i - 1]].position.y;
        trajectory[keys[i]].position.z = trajectory[keys[i - 1]].position.z;
        trajectory[keys[i]].orientation.w = trajectory[keys[i - 1]].orientation.w;
        trajectory[keys[i]].orientation.x = trajectory[keys[i - 1]].orientation.x;
        trajectory[keys[i]].orientation.y = trajectory[keys[i - 1]].orientation.y;
        trajectory[keys[i]].orientation.z = trajectory[keys[i - 1]].orientation.z;
      }
    }
  }
}

void HumanTrajectory::interpolate(double step)
{
  for (unsigned int i = 0; i < keys.size() - 1; i ++)
  {
    if (!comparePoses(trajectory[keys[i]], trajectory[keys[i + 1]]))
    {
      // setup all of the data we need (start and endpoints, steps, etc.)
      double t1 = keys[i];
      double t2 = keys[i + 1];
      geometry_msgs::Pose p1 = trajectory[t1];
      geometry_msgs::Pose p2 = trajectory[t2];

      double dt = keys[i + 1] - keys[i];
      int n = (int)(dt/step);

      double dx = p2.position.x - p1.position.x;
      double dy = p2.position.y - p1.position.y;
      double dz = p2.position.z - p1.position.z;

      btQuaternion q1(p1.orientation.x, p1.orientation.y, p1.orientation.z, p1.orientation.w);
      btQuaternion q2(p2.orientation.x, p2.orientation.y, p2.orientation.z, p2.orientation.w);

      // perform interpolation
      geometry_msgs::Pose addPose;
      for (int j = 1; j <= n; j ++)
      {
        double tj = t1 + j*step;
        double percentage = j*step/dt;

        // interpolate position
        addPose.position.x = p1.position.x + percentage*dx;
        addPose.position.y = p1.position.y + percentage*dy;
        addPose.position.z = p1.position.z + percentage*dz;

        // interpolate (slerp) rotation
        btQuaternion q_slerp = q1.slerp(q2, percentage);
        addPose.orientation.w = q_slerp.w();
        addPose.orientation.x = q_slerp.x();
        addPose.orientation.y = q_slerp.y();
        addPose.orientation.z = q_slerp.z();

        keys.insert(keys.begin() + i + 1, tj);
        trajectory.insert(std::pair<double, geometry_msgs::Pose>(tj, addPose));
        i ++;
      }
    }
  }
}

geometry_msgs::Pose HumanTrajectory::getPose(double t)
{
  std::vector<double>::iterator it = std::upper_bound(keys.begin(), keys.end(), t);

  if (it != keys.begin())
  {
    return trajectory[*(it - 1)];
  }
  return trajectory[*it];
}

bool HumanTrajectory::comparePoses(geometry_msgs::Pose p1, geometry_msgs::Pose p2)
{
  return p1.position.x == p2.position.x
    && p1.position.y == p2.position.y
    && p1.position.z == p2.position.z
    && p1.orientation.w == p2.orientation.w
    && p1.orientation.x == p2.orientation.x
    && p1.orientation.y == p2.orientation.y
    && p1.orientation.z == p2.orientation.z;
}