
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
  group.setStartStateToCurrentState();




  std::vector<std::vector<double>> waypoints;

  std::vector<double> joint_conf1(6,0); // [0,0,0,0,0,0]
  waypoints.push_back(joint_conf1);

  std::vector<double> joint_conf2(6,1); // [1,1,1,1,1,1]
  waypoints.push_back(joint_conf2);

  std::vector<double> joint_conf3(6,-1); // [-1,-1,-1,-1,-1,-1]
  waypoints.push_back(joint_conf3);

  moveit_msgs::RobotTrajectory trajectory_msg;
  group.setPlanningTime(10.0);

  moveit::core::RobotState trj_state = *group.getCurrentState();

  robot_trajectory::RobotTrajectory trajectory(group.getRobotModel(), group.getName());
  trajectory_msgs::JointTrajectory complete_trj;
  moveit::planning_interface::MoveGroupInterface::Plan complete_plan;

  for (unsigned int iw=0;iw<waypoints.size();iw++)
  {
    ROS_INFO("computing waypoint %u",iw);
    group.setJointValueTarget(waypoints.at(iw));

    moveit::planning_interface::MoveGroupInterface::Plan partial_plan;

    bool success = (group.plan(partial_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (!success)
    {
      ROS_ERROR("Planning failed computing waypoint %u", iw);
      return 0;
    }

    if (iw==0)
    {
      // store starting point of the first plan
      complete_plan.start_state_=partial_plan.start_state_;
      // store joint name
      complete_trj.joint_names=partial_plan.trajectory_.joint_trajectory.joint_names;
    }

    //append trajectory point removing time
    for (unsigned int ip=0;ip<partial_plan.trajectory_.joint_trajectory.points.size();ip++)
    {

      complete_trj.points.push_back(partial_plan.trajectory_.joint_trajectory.points.at(ip));
      complete_trj.points.back().time_from_start=ros::Duration(0);
    }

    // set start start for the next planning equal to actual waypoint
    trj_state.setJointGroupPositions(group.getName(),waypoints.at(iw));
    group.setStartState(trj_state);

  }
  // recompute time without stop in waypoints
  trajectory.setRobotTrajectoryMsg(trj_state, complete_trj);

  trajectory_processing::IterativeParabolicTimeParameterization ipt;
  double velocity_scaling=1; // between 0 and 1
  double acceleration_scaling=1; // between 0 and 1
  if (!ipt.computeTimeStamps(trajectory,
                             velocity_scaling,
                             acceleration_scaling))
  {
    ROS_FATAL("failing compute time");
    return 0;
  }

  // save complete trajectory to a plan
  trajectory.getRobotTrajectoryMsg(complete_plan.trajectory_);

  ros::Duration(3).sleep(); // wait 3 seconds to be able to see the movement in rviz
  group.execute(complete_plan);

	ros::shutdown();
	return 0;
}


