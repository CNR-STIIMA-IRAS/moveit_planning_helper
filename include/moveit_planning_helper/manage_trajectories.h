#ifndef MANAGE_TRJS_MOVEIT_HELPER___
#define MANAGE_TRJS_MOVEIT_HELPER___

#include <rosparam_utilities/rosparam_utilities.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_spline_parameterization.h>
#include <eigen_matrix_utils/eigen_matrix_utils.h>
#include <moveit_planning_helper/spline_interpolator.h>
#include <name_sorting/sort_trajectories.h>

namespace trajectory_processing
{


inline bool getTrajectoryFromParam(const std::string& ns, const std::string& trj_name, trajectory_msgs::JointTrajectory& trj, std::string& what)
{
  std::string err;
  std::vector<std::vector<double>> positions;

  if (!rosparam_utilities::get(ns + "/"+trj_name+"/positions", positions, what))
  {
    return false;
  }
  unsigned int npnt = positions.size();

  std::vector<std::vector<double>> velocities;
  rosparam_utilities::get(ns + "/" + trj_name+"/velocities", velocities, err);
  what += err + (err.length()>0 ? "\n" : "");
  // == == == == == == == == == == == == == == 
  if (npnt == 0)
  {
    what += trj_name + "/positions with no points";
    return false;
  }
  int dimension = positions.at(0).size();
  if (npnt == 0)
  {
    what += trj_name + "/positions with no dimensions";
    return false;
  }
  // == == == == == == == == == == == == == == 
  std::vector<double> time(positions.size(),0);
  rosparam_utilities::get(ns + "/" + trj_name+"/time_from_start", time, err, &time);
  what += err + (err.length()>0 ? "\n" : "");

  if (time.size() != npnt)
  {
    what += trj_name + "/time_from_start has wrong dimensions";
    return false;
  }
  trj.points.resize(npnt);

  trj.joint_names.resize(dimension);
  if (!rosparam_utilities::get(ns + "/" +trj_name+"/joint_names",trj.joint_names, err))
  {
    what += err + (err.length()>0 ? "\n" : "");
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


[[deprecated]]
inline bool getTrajectoryFromParam(const ros::NodeHandle& nh, const std::string& trj_name, trajectory_msgs::JointTrajectory& trj)
{
  std::string what; 
  bool ret = getTrajectoryFromParam(nh.getNamespace(), trj_name, trj, what);
  if(!ret)
  {
    ROS_ERROR("%s", what.c_str());
  }
  return ret;
}



inline bool setTrajectoryToParam(const std::string& ns, const std::string& trj_name, const trajectory_msgs::JointTrajectory& trj, std::string& what)
{
  if (trj.points.size()==0)
  {
    what = "Trajectory with no points";
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

  rosparam_utilities::set(ns + "/" +trj_name+"/joint_names",trj.joint_names,what);
  rosparam_utilities::set(ns + "/" +trj_name+"/time_from_start",time,what);


  if(!rosparam_utilities::set(ns + "/" + trj_name+"/positions",positions,what)
  ||  !rosparam_utilities::set(ns + "/" + trj_name+"/velocities",velocities,what)
  ||  !rosparam_utilities::set(ns + "/" + trj_name+"/accelerations",accelerations,what)
  ||  !rosparam_utilities::set(ns + "/" + trj_name+"/effort",effort,what))
    return false;

  return true;
}

[[deprecated]]
inline bool setTrajectoryToParam(const ros::NodeHandle& nh, const std::string& trj_name, const trajectory_msgs::JointTrajectory& trj)
{
  std::string what; 
  bool ret = setTrajectoryToParam(nh.getNamespace(), trj_name,trj, what);
  if(!ret)
  {
    ROS_ERROR("%s", what.c_str());
  }
  return ret;
}

bool checkCollisionBetweenTrajectories(const robot_trajectory::RobotTrajectory& trj1, const robot_trajectory::RobotTrajectory& trj2, const std  ::string& joined_group_name);

double computeTrajectoryLength(const trajectory_msgs::JointTrajectory& trj);

}


#endif
