#ifndef MANAGE_TRJS_MOVEIT_HELPER___
#define MANAGE_TRJS_MOVEIT_HELPER___

#include <rosparam_utilities/rosparam_utilities.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit_planning_helper/iterative_spline_parameterization.h>
#include <eigen_matrix_utils/eigen_matrix_utils.h>
#include <moveit_planning_helper/spline_interpolator.h>
#include <name_sorting/sort_trajectories.h>

namespace trajectory_processing
{

inline bool getTrajectoryFromParam(const ros::NodeHandle& nh, const std::string& trj_name, trajectory_msgs::JointTrajectory& trj)
{
  std::vector<std::vector<double>> positions;

  if (!rosparam_utilities::getParamMatrix(nh, trj_name+"/positions", positions))
  {
    ROS_ERROR("%s/positions does not exist", trj_name.c_str());
    return false;
  }
  unsigned int npnt = positions.size();

  std::vector<std::vector<double>> velocities;
  if (!rosparam_utilities::getParamMatrix(nh, trj_name+"/velocities", velocities))
  {
    ROS_DEBUG("%s/velocities does not exist", trj_name.c_str());
  }

  if (npnt == 0)
  {
    ROS_ERROR("%s/positions with no points", trj_name.c_str());
    return -1;
  }
  int dimension = positions.at(0).size();
  if (npnt == 0)
  {
    ROS_ERROR("%s/positions with no dimensions", trj_name.c_str());
    return false;
  }

  std::vector<double> time(positions.size(),0);
  if (!nh.getParam(trj_name+"/time_from_start", time))
  {
    ROS_DEBUG("%s/time_from_start does not exist", trj_name.c_str());

  }
  if (time.size() != npnt)
  {
    ROS_ERROR("%s/time_from_start has wrong dimensions", trj_name.c_str());
    return false;
  }
  trj.points.resize(npnt);

  trj.joint_names.resize(dimension);
  if (!nh.getParam(trj_name+"/joint_names",trj.joint_names))
  {
    for (int idx=0;idx<dimension;idx++)
      trj.joint_names.at(idx)= "joint_"+std::to_string(idx);
  }

  for (unsigned int iPnt = 0;iPnt<npnt;iPnt++)
  {
    trj.points.at(iPnt).positions.resize(dimension);
    trj.points.at(iPnt).positions = positions.at(iPnt);
    if (velocities.size()>0)
    {
      trj.points.at(iPnt).velocities.resize(dimension);
      trj.points.at(iPnt).velocities = velocities.at(iPnt);
    }
    trj.points.at(iPnt).accelerations.resize(dimension,0);
    trj.points.at(iPnt).effort.resize(dimension,0);

    trj.points.at(iPnt).time_from_start = ros::Duration(time.at(iPnt));
  }
  return true;
}

inline bool setTrajectoryToParam( ros::NodeHandle& nh, const std::string& trj_name, const trajectory_msgs::JointTrajectory& trj)
{
  if (trj.points.size()==0)
  {
    ROS_WARN("Trajectory with no points");
    return false;
  }

  std::vector<std::vector<double>> positions(trj.points.size());
  std::vector<std::vector<double>> velocities(trj.points.size());
  std::vector<std::vector<double>> accelerations(trj.points.size());
  std::vector<std::vector<double>> effort(trj.points.size());
  std::vector<double> time(trj.points.size());

  for (unsigned int iPnt=0; iPnt<trj.points.size(); iPnt++)
  {
    positions.at(iPnt) = trj.points.at(iPnt).positions;
    velocities.at(iPnt) = trj.points.at(iPnt).velocities;
    accelerations.at(iPnt) = trj.points.at(iPnt).accelerations;
    effort.at(iPnt) = trj.points.at(iPnt).effort;
    time.at(iPnt) = trj.points.at(iPnt).time_from_start.toSec();
  }

  nh.setParam(trj_name+"/joint_names",trj.joint_names);
  nh.setParam(trj_name+"/time_from_start",time);


  if (!rosparam_utilities::setParam<double>(nh,trj_name+"/positions",positions))
    return false;
  if (!rosparam_utilities::setParam(nh,trj_name+"/velocities",velocities))
    return false;
  if (!rosparam_utilities::setParam(nh,trj_name+"/accelerations",accelerations))
    return false;
  if (!rosparam_utilities::setParam(nh,trj_name+"/effort",effort))
    return false;

  return true;
}

inline trajectory_msgs::JointTrajectory createDenseTrajectory(const trajectory_msgs::JointTrajectory& trj, double sampling_period);

bool checkCollisionBetweenTrajectories(const robot_trajectory::RobotTrajectory& trj1, const robot_trajectory::RobotTrajectory& trj2, const std::__cxx11::string& joined_group_name);

double computeTrajectoryLength(const trajectory_msgs::JointTrajectory& trj);

}


#endif
