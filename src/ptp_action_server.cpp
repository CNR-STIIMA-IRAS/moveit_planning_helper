#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <moveit_msgs/ExecuteTrajectoryAction.h>
#include <boost/lexical_cast.hpp>
#include <std_srvs/Empty.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>


std::shared_ptr<moveit::planning_interface::MoveGroupInterface> group;
std::shared_ptr<actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>> ptp_as;
void ptpGoalCb(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal)
{
//  goal->point;
//  group.setJointValueTarget(waypoints.at(iw));
  control_msgs::FollowJointTrajectoryResult result;
  if (goal->trajectory.points.size()!=1)
  {
    ROS_ERROR("point to point required only one trajectory point");
    result.error_code=control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;
    ptp_as->setAborted(result);
    return;
  }

  group->setJointValueTarget(goal->trajectory.points.at(0).positions);
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = (group->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (!success)
  {
    ROS_ERROR("unable to plan to goal");
    result.error_code=control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;
    ptp_as->setAborted(result);
    return;
  }

  if (group->move())
  {
    result.error_code=control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
    ptp_as->setSucceeded(result);
    return;
  }
  else
  {
    result.error_code=control_msgs::FollowJointTrajectoryResult::PATH_TOLERANCE_VIOLATED;
    ptp_as->setSucceeded(result);
    return;
  }

}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "ptp_action_server");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(4);
  spinner.start();

  std::string group_name;
  if (!nh.getParam("group_name",group_name))
  {
    ROS_ERROR("no group_name specified");
    return 0;
  }
  group=std::make_shared<moveit::planning_interface::MoveGroupInterface>(group_name);

  ptp_as=std::make_shared<actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>>(group_name+"/ptp_joint_trajectory",&ptpGoalCb,false);
  ptp_as->start();
  ros::Rate lp(100);
  while (ros::ok())
    lp.sleep();

  return 0;
}



