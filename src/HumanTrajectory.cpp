#include "waypoint_planner/HumanTrajectory.h"

HumanTrajectory::HumanTrajectory()
{

}

void HumanTrajectory::addPose(double time, geometry_msgs::PoseStamped pose)
{
  trajectory.insert(std::pair<double, geometry_msgs::PoseStamped>(time, pose));
  keys.push_back(time);
}

void HumanTrajectory::sortKeys()
{
  std::sort(keys.begin(), keys.end());
}

void HumanTrajectory::interpolate(double step)
{
  for (unsigned int i = 0; i < keys.size() - 2; i ++)
  {
    if (!comparePoses(trajectory[keys[i]], trajectory[keys[i + 1]]))
    {
      // setup all of the data we need (start and endpoints, steps, etc.)
      double t1 = keys[i];
      double t2 = keys[i + 1];
      geometry_msgs::Pose p1 = trajectory[t1].pose;
      geometry_msgs::Pose p2 = trajectory[t2].pose;

      double dt = keys[i + 1] - keys[i];
      int n = (int)(dt/step);

      double dx = p2.position.x - p1.position.x;
      double dy = p2.position.y - p1.position.y;
      double dz = p2.position.z - p1.position.z;

      btQuaternion q1(p1.orientation.x, p1.orientation.y, p1.orientation.z, p1.orientation.w);
      btQuaternion q2(p2.orientation.x, p2.orientation.y, p2.orientation.z, p2.orientation.w);

      // perform interpolation
      geometry_msgs::PoseStamped addPose;
      addPose.header.frame_id = trajectory[t1].header.frame_id;
      for (int j = 1; j <= n; j ++)
      {
        double tj = t1 + j*step;
        double percentage = j*step/dt;

        // interpolate position
        addPose.pose.position.x = p1.position.x + percentage*dx;
        addPose.pose.position.y = p1.position.y + percentage*dy;
        addPose.pose.position.z = p1.position.z + percentage*dz;

        // interpolate (slerp) rotation
        btQuaternion q_slerp = q1.slerp(q2, percentage);
        addPose.pose.orientation.w = q_slerp.w();
        addPose.pose.orientation.x = q_slerp.x();
        addPose.pose.orientation.y = q_slerp.y();
        addPose.pose.orientation.z = q_slerp.z();

        keys.insert(keys.begin() + i + 1, tj);
        trajectory.insert(std::pair<double, geometry_msgs::PoseStamped>(tj, addPose));
        i ++;
      }
    }
  }
}

geometry_msgs::PoseStamped HumanTrajectory::getPose(double t)
{
  std::vector<double>::iterator it = std::upper_bound(keys.begin(), keys.end(), t);

  if (it != keys.begin())
  {
    return trajectory[*(it - 1)];
  }
  return trajectory[*it];
}

bool HumanTrajectory::comparePoses(geometry_msgs::PoseStamped p1, geometry_msgs::PoseStamped p2)
{
  return p1.header.frame_id == p2.header.frame_id
    && p1.pose.position.x == p2.pose.position.x
    && p1.pose.position.y == p2.pose.position.y
    && p1.pose.position.z == p2.pose.position.z
    && p1.pose.orientation.w == p2.pose.orientation.w
    && p1.pose.orientation.x == p2.pose.orientation.x
    && p1.pose.orientation.y == p2.pose.orientation.y
    && p1.pose.orientation.z == p2.pose.orientation.z;
}