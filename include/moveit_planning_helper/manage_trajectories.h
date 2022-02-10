#ifndef MANAGE_TRJS_MOVEIT_HELPER___
#define MANAGE_TRJS_MOVEIT_HELPER___

#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_spline_parameterization.h>

namespace trajectory_processing
{

/**
 * @brief Get the Trajectory From Param Server
 * 
 * @param[in] full_path_to_trj full path to the trajectory in a yaml structure. The first charcater must be '/', otherwise an error is returned
 * @param[out] trj list of points
 * @param[out] what if an error or warning is generated
 * @return true 
 * @return false 
 */
bool getTrajectoryFromParam(const std::string& full_path_to_trj, trajectory_msgs::JointTrajectory& trj, std::string& what);

/**
 * @brief Get the Trajectory From Param Server
 * 
 * @param nh 
 * @param trj_name 
 * @param trj 
 * @return true 
 * @return false 
 */
[[deprecated("getTrajectoryFromParam(const std::string&, ...)")]]
 bool getTrajectoryFromParam(const ros::NodeHandle& nh, const std::string& trj_name, trajectory_msgs::JointTrajectory& trj);

/**
 * @brief Set the Trajectory To Param object
 * 
 * @param full_path_to_trj
 * @param trj 
 * @param what 
 * @return true 
 * @return false 
 */
bool setTrajectoryToParam(const std::string& full_path_to_trj,  const trajectory_msgs::JointTrajectory& trj, std::string& what);

/**
 * @brief Set the Trajectory To Param object
 * 
 * @param nh 
 * @param trj_name 
 * @param trj 
 * @return true 
 * @return false 
 */
[[deprecated("setTrajectoryToParam(const std::string&, ...)")]]
bool setTrajectoryToParam(const ros::NodeHandle& nh, const std::string& trj_name, const trajectory_msgs::JointTrajectory& trj);

/**
 * @brief 
 * 
 * @param trj1 
 * @param trj2 
 * @param joined_group_name 
 * @return true 
 * @return false 
 */
bool checkCollisionBetweenTrajectories(const robot_trajectory::RobotTrajectory& trj1, const robot_trajectory::RobotTrajectory& trj2, const std  ::string& joined_group_name);

/**
 * @brief 
 * 
 * @param trj 
 * @return double 
 */
double computeTrajectoryLength(const trajectory_msgs::JointTrajectory& trj);

}

#endif
