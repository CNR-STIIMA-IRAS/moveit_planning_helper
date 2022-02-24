#include <moveit_planning_helper/manage_trajectories.h>

namespace trajectory_processing 
{



bool checkCollisionBetweenTrajectories(const robot_trajectory::RobotTrajectory& trj1, const robot_trajectory::RobotTrajectory& trj2, const std::string& joined_group_name)
{

  if (!trj1.getGroupName().compare(trj2.getGroupName()))
  {
    ROS_ERROR("Robot trajectories should be of two different move gruop");
    return false;
  }
  
  assert(0);
  
  return false;
}

double computeTrajectoryLength(const trajectory_msgs::JointTrajectory &trj)
{
  if (trj.points.size()==0)
    return 0;

  double length=0;
  std::vector<double> last_pos=trj.points.at(0).positions;
  for (unsigned int idx=1;idx<trj.points.size();idx++)
  {
    std::vector<double> pos=trj.points.at(idx).positions;
    double squared_segment_length=0;
    for (unsigned int iax=0;iax<pos.size();iax++)
    {
      squared_segment_length+=std::pow(pos.at(iax)-last_pos.at(iax),2);
    }
    length+=std::sqrt(squared_segment_length);
  }
  return length;
}
}
