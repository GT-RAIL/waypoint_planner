#include "waypoint_planner/HumanTrajectory.h"

using std::vector;

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

//TODO: add intermediate points first, spline and interpolate in a separate function
void HumanTrajectory::perturbTrajectory(double chance)
{
  long seed = std::chrono::system_clock::now().time_since_epoch().count();
  std::mt19937 gen(seed);  // random seed

  std::uniform_real_distribution<double> perturb_chance_dst(0, 1);

  sortKeys();

  for (size_t i = 0; i < keys.size() - 1; i ++)
  {
    if (comparePoses(trajectory[keys[i]], trajectory[keys[i + 1]]))
    {
      continue;
    }

    double t1 = keys[i];
    double t2 = keys[i + 1];
    double segment_length = t2 - t1;
    geometry_msgs::Pose p1 = trajectory[t1];
    geometry_msgs::Pose p2 = trajectory[t2];
    if (segment_length >= 10)
    {
      if (perturb_chance_dst(gen) > chance)
      {
        std::cout << "skipping perturbation." << std::endl;
        continue;
      }
      else
      {
        std::cout << "perturbing arc." << std::endl;
      }

      std::uniform_real_distribution<double> time_dst(1, t2 - t1 - 1);
      double dt = time_dst(gen);

      // perform interpolation
      geometry_msgs::Pose add_pose;
      double percentage = dt/segment_length;

      // interpolate position
      add_pose.position.x = p1.position.x + percentage*(p2.position.x - p1.position.x);
      add_pose.position.y = p1.position.y + percentage*(p2.position.y - p1.position.y);
      add_pose.position.z = p1.position.z + percentage*(p2.position.z - p1.position.z);

      // interpolate (slerp) rotation
      btQuaternion q1(p1.orientation.x, p1.orientation.y, p1.orientation.z, p1.orientation.w);
      btQuaternion q2(p2.orientation.x, p2.orientation.y, p2.orientation.z, p2.orientation.w);
      btQuaternion q_slerp = q1.slerp(q2, percentage);
      add_pose.orientation.w = q_slerp.w();
      add_pose.orientation.x = q_slerp.x();
      add_pose.orientation.y = q_slerp.y();
      add_pose.orientation.z = q_slerp.z();

      // randomize the intermediate keypoint
      add_pose.position.x += (2*static_cast<double>(rand())/RAND_MAX - 1)*.08*segment_length;
      add_pose.position.y += (2*static_cast<double>(rand())/RAND_MAX - 1)*.08*segment_length;
      add_pose.position.z += (2*static_cast<double>(rand())/RAND_MAX - 1)*.08*segment_length;
      keys.insert(keys.begin() + i + 1, t1 + dt);
      trajectory.insert(std::pair<double, geometry_msgs::Pose>(t1 + dt, add_pose));
      i ++;

//        btQuaternion q_add(addPose.orientation.x, addPose.orientation.y, addPose.orientation.z, addPose.orientation.w);
//        btQuaternion q_rand;
//        q_rand.setEuler((2*static_cast<double>(rand())/RAND_MAX - 1)*arc_rot,
//                        (2*static_cast<double>(rand())/RAND_MAX - 1)*arc_rot,
//                        (2*static_cast<double>(rand())/RAND_MAX - 1)*arc_rot);
//        q_add = q_add*q_rand;
//        addPose.orientation.w = q_add.w();
//        addPose.orientation.x = q_add.x();
//        addPose.orientation.y = q_add.y();
//        addPose.orientation.z = q_add.z();
    }
  }
}

void HumanTrajectory::splineTrajectory(double step)
{
  long seed = std::chrono::system_clock::now().time_since_epoch().count();
  std::mt19937 gen(seed);  // random seed

  size_t last_key = keys.size() - 1;
  bool no_move = false;

  for (size_t i = 0; i < last_key; i ++)
  {
    if (comparePoses(trajectory[keys[i]], trajectory[keys[i + 1]]))
    {
      continue;
    }
    vector<double> time_pts;
    vector<geometry_msgs::Pose> pos_pts;
    time_pts.push_back(keys[i]);
    pos_pts.push_back(trajectory[keys[i]]);
    geometry_msgs::Point prev_pt = pos_pts[0].position;
    for (size_t j = i + 1; j <= last_key; j ++)
    {
      if (trajectory[keys[j]].position.x == prev_pt.x && trajectory[keys[j]].position.y == prev_pt.y
          && trajectory[keys[j]].position.z == prev_pt.z)
      {
        break;
      }
      time_pts.push_back(keys[j]);
      pos_pts.push_back(trajectory[keys[j]]);
      prev_pt = trajectory[keys[j]].position;
    }

    ecl::Array<double> time_points(pos_pts.size());
    ecl::Array<double> x_points(pos_pts.size());
    ecl::Array<double> y_points(pos_pts.size());
    ecl::Array<double> z_points(pos_pts.size());
    for (size_t j = 0; j < pos_pts.size(); j ++)
    {
      time_points[j] = time_pts[j];
      x_points[j] = pos_pts[j].position.x;
      y_points[j] = pos_pts[j].position.y;
      z_points[j] = pos_pts[j].position.z;
    }

    // generate spline
    ecl::CubicSpline x_spline, y_spline, z_spline;
    x_spline = ecl::CubicSpline::Natural(time_points, x_points);
    y_spline = ecl::CubicSpline::Natural(time_points, y_points);
    z_spline = ecl::CubicSpline::Natural(time_points, z_points);

    // add splined interpolated poses to the trajectory
    for (size_t j = 0; j < pos_pts.size() - 1; j ++)
    {
      // setup all of the data we need (start and endpoints, steps, etc.)
      double t1 = time_pts[j];
      double t2 = time_pts[j + 1];
      geometry_msgs::Pose p1 = pos_pts[j];
      geometry_msgs::Pose p2 = pos_pts[j + 1];

      double dt = t2 - t1;
      int n = (int)(dt/step);

      double dx = p2.position.x - p1.position.x;
      double dy = p2.position.y - p1.position.y;
      double dz = p2.position.z - p1.position.z;

      btQuaternion q1(p1.orientation.x, p1.orientation.y, p1.orientation.z, p1.orientation.w);
      btQuaternion q2(p2.orientation.x, p2.orientation.y, p2.orientation.z, p2.orientation.w);

      // perform interpolation
      geometry_msgs::Pose addPose;
      for (int k = 1; k <= n; k ++)
      {
        double tk = t1 + k*step;
        double percentage = k*step/dt;

        // get splined position
        addPose.position.x = x_spline(tk);
        addPose.position.y = y_spline(tk);
        addPose.position.z = z_spline(tk);

        // interpolate (slerp) rotation
        btQuaternion q_slerp = q1.slerp(q2, percentage);
        addPose.orientation.w = q_slerp.w();
        addPose.orientation.x = q_slerp.x();
        addPose.orientation.y = q_slerp.y();
        addPose.orientation.z = q_slerp.z();

        keys.push_back(tk);
        trajectory.insert(std::pair<double, geometry_msgs::Pose>(tk, addPose));
      }
    }
  }

  sortKeys();
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

void HumanTrajectory::fromMsg(const waypoint_planner::HumanTrajectoryMsg &msg)
{
  for (size_t i = 0; i < msg.times.size(); i ++)
  {
    addPose(msg.times[i], msg.poses[i]);
  }
  sortKeys();
}

waypoint_planner::HumanTrajectoryMsg HumanTrajectory::toMsg()
{
  waypoint_planner::HumanTrajectoryMsg msg;
  msg.times.resize(keys.size());
  msg.poses.resize(keys.size());
  for (size_t i = 0; i < keys.size(); i ++)
  {
    msg.times[i] = keys[i];
    msg.poses[i] = trajectory[keys[i]];
  }
  return msg;
}
