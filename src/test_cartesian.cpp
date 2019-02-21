
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <moveit/robot_state/conversions.h>

#include <actionlib/client/simple_action_client.h>
#include <moveit_msgs/ExecuteTrajectoryAction.h>
#include <boost/lexical_cast.hpp>
#include <std_srvs/Empty.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

int main(int argc, char **argv)
{

  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  moveit::planning_interface::MoveGroupInterface group("ur10");

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  group.setStartStateToCurrentState();

  std::vector<geometry_msgs::Pose> waypoints;

  // as example I take the actual cartesian pose
  geometry_msgs::Pose target_pose = group.getCurrentPose().pose;

  //and add a small movement
  target_pose.position.x += 0.05;
  target_pose.position.y += 0.05;
  target_pose.position.z += 0.05;
  //insert point in the waypoints vector
  waypoints.push_back(target_pose);

  target_pose.position.x += 0.00;
  target_pose.position.y += -0.05;
  target_pose.position.z += 0.00;
  //insert point in the waypoints vector
  waypoints.push_back(target_pose);

  target_pose.position.x += -0.05;
  target_pose.position.y += 0.00;
  target_pose.position.z += -0.05;
  //insert point in the waypoints vector
  waypoints.push_back(target_pose);

  moveit_msgs::RobotTrajectory trajectory_msg;
  group.setPlanningTime(10.0);

  double fraction = group.computeCartesianPath(waypoints,
                                               0.01,  // eef_step
                                               0.0,   // jump_threshold
                                               trajectory_msg,
                                               false);

  ROS_INFO("moveit was able to compute %f%% of the path",fraction*100);
  // The trajectory needs to be modified so it will include velocities as well.
  // First to create a RobotTrajectory object
  robot_trajectory::RobotTrajectory rt(group.getCurrentState()->getRobotModel(), group.getName());

  // Second get a RobotTrajectory from trajectory
  rt.setRobotTrajectoryMsg(*group.getCurrentState(), trajectory_msg);

  // Thrid create a IterativeParabolicTimeParameterization object
  trajectory_processing::IterativeParabolicTimeParameterization iptp;
  // Fourth compute computeTimeStamps
  bool success = iptp.computeTimeStamps(rt);
  ROS_INFO("Computed time stamp %s",success?"SUCCEDED":"FAILED");
  // Get RobotTrajectory_msg from RobotTrajectory
  rt.getRobotTrajectoryMsg(trajectory_msg);


  plan.trajectory_ = trajectory_msg;

  group.execute(plan);

	ros::shutdown();
	return 0;
}



