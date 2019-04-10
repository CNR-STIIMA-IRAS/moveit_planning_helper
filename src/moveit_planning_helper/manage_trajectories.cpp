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

}
