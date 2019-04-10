#include <moveit_planning_helper/manage_trajectories.h>
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



int main(int argc, char **argv)
{
  ros::init(argc, argv, "execute_trj_moveit");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  actionlib::SimpleActionClient<moveit_msgs::ExecuteTrajectoryAction> ac(nh,"/execute_trajectory");
  if (!ac.waitForServer(ros::Duration(10)))
  {
    ROS_ERROR("NO /execute_trajectory server");
    return 0;
  }
  std::string group_name;
  if (!nh.getParam("group_name",group_name))
    group_name="iiwa14";

  moveit::planning_interface::MoveGroupInterface move_group(group_name);
  ROS_INFO_NAMED("tutorial", "Reference frame: %s", move_group.getPlanningFrame().c_str());
  ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());
  std::vector<std::string> js= move_group.getActiveJoints();
  ROS_INFO_NAMED("tutorial", "Controlled joint");
  for (std::string& s: js)
    ROS_INFO_NAMED("tutorial", "- %s", s.c_str());

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success;

  std::string trj_name;
  if (!nh.getParam("trj_name",trj_name))
  {
    ROS_ERROR("NO trj_name specified");
    return -1;
  }

  std::vector<std::string> executed_trjs;

  if (!trj_name.compare("load"))
  {
    std::vector<std::string> list_trjs;
    if (!nh.getParam("/list_of_trajectories",list_trjs))
    {
      ROS_ERROR("NO list_of_trajectories specified");
      return -1;
    }
    ROS_INFO("select trajectory:");
    ROS_INFO("-10) execute all the trajectories");
    for (unsigned int idx=0;idx<list_trjs.size();idx++)
      ROS_INFO("%d) %s",idx,list_trjs.at(idx).c_str());

    std::cout <<"enter number: ";
    std::string input;
    std::cin>>input;

    int x;
    try
    {
      x = boost::lexical_cast<int>( input );
    }
    catch( boost::bad_lexical_cast const& )
    {
      std::cout << "Error: input string was not valid" << std::endl;
      return -1;
    }
    if (x==-10)
    {
      ROS_INFO("execute all the trajectories");
      executed_trjs=list_trjs;
    }
    else if ((x<0) || (x>=list_trjs.size()))
    {
      ROS_ERROR("out of bound");
      return -1;
    }
    else
    {
      ROS_INFO("selected %d) %s",x,list_trjs.at(x).c_str());
      trj_name=list_trjs.at(x).c_str();
      executed_trjs.push_back(trj_name);
    }
  }
  else
    executed_trjs.push_back(trj_name);

  for (auto trj_name: executed_trjs)
  {
    ros::Time t0=ros::Time::now();
    nh.setParam("/binary_logger/test_name",trj_name+"__"+std::to_string(t0.sec));
    ros::ServiceClient start_log = nh.serviceClient<std_srvs::Empty>("/start_log");
    ros::ServiceClient stop_log  = nh.serviceClient<std_srvs::Empty>("/stop_log");


    trajectory_msgs::JointTrajectory trj;

    if (!trajectory_processing::getTrajectoryFromParam(nh, trj_name, trj))
    {
      ROS_ERROR("%s not found",trj_name.c_str());
      return 0;
    }

    // Next get the current set of joint values for the group.
    std::vector<double> initial_position=trj.points.begin()->positions;
    std::map<std::string,double> ini_pos;
    for (unsigned int iJoint=0;iJoint<initial_position.size();iJoint++)
      ini_pos.insert(std::pair<std::string,double>(trj.joint_names.at(iJoint),initial_position.at(iJoint)));
    move_group.setJointValueTarget(ini_pos);
    success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (!success)
    {
      ROS_ERROR("Planning failed");
      return 0;
    }

    ROS_INFO("Move IIWA to initial position");
    moveit::planning_interface::MoveItErrorCode err=move_group.move();
    if (err!= moveit_msgs::MoveItErrorCodes::SUCCESS)
    {
      ROS_ERROR("moveit error");
      return 0;
    }
    ros::Duration(3).sleep();

    std_srvs::Empty srv;
    start_log.call(srv);
    ros::Duration(1).sleep();
    moveit_msgs::ExecuteTrajectoryGoal goal;
    ROS_INFO("Execute trajectory %s",trj_name.c_str());
    goal.trajectory.joint_trajectory=trj;
    ac.sendGoalAndWait(goal);
    ros::Duration(1).sleep();
    stop_log.call(srv);
    ros::Duration(1).sleep();
  }

  ros::shutdown();
  return 0;
}
