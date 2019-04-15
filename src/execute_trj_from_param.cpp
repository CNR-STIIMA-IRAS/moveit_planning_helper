
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

#include <moveit_planning_helper/manage_trajectories.h>

int main(int argc, char **argv)
{

  ros::init(argc, argv, "iiwa_moveit");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  actionlib::SimpleActionClient<moveit_msgs::ExecuteTrajectoryAction> ac(nh, "/execute_trajectory");
  if (!ac.waitForServer(ros::Duration(10)))
  {
    ROS_ERROR("NO /execute_trajectory server");
    return 0;
  }
  std::string group_name;
  if (!nh.getParam("group_name", group_name))
  {
    group_name = "manipulator";
  }

  moveit::planning_interface::MoveGroupInterface move_group(group_name);
  ROS_INFO("Reference frame: %s", move_group.getPlanningFrame().c_str());
  ROS_INFO("End effector link: %s", move_group.getEndEffectorLink().c_str());

  std::vector<std::string> js = move_group.getActiveJoints();
  ROS_INFO("Controlled joint");
  for (std::string & s : js)
  {
    ROS_INFO("- %s", s.c_str());
  }

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success;

  std::string trj_name;
  if (!nh.getParam("trj_name", trj_name))
  {
    ROS_INFO("NO trj_name specified, loadind /list_of_trajectories");
    trj_name = "load";
  }

  std::vector<std::string> executed_trjs;
  int repetions;
  if (!nh.getParam("/binary_logger/log_repetition", repetions))
  {
    repetions = 1;
  }

  if (!trj_name.compare("load"))
  {
    ROS_INFO("------------------");
    std::vector<std::string> list_trjs;
    if (!nh.getParam("/list_of_trajectories", list_trjs))
    {
      ROS_ERROR("NO list_of_trajectories specified");
      return -1;
    }

    ROS_INFO("select trajectory:");
    ROS_INFO("-10) execute all the trajectories");
    ROS_INFO("-9) to set repetions number (now is %d)", repetions);
    for (unsigned int idx = 0; idx < list_trjs.size(); idx++)
    {
      ROS_INFO("%d) %s", idx, list_trjs.at(idx).c_str());
    }

    std::cout << "enter number: ";
    std::string input;
    std::cin >> input;

    int x;
    try
    {
      x = boost::lexical_cast<int> (input);
    }
    catch (boost::bad_lexical_cast const &)
    {
      std::cout << "Error: input string was not valid" << std::endl;
      return -1;
    }
    if (x == -10)
    {
      ROS_INFO("execute all the trajectories");
      executed_trjs = list_trjs;
    }
    else if (x == -9)
    {
      std::cout << "set number of repetions: ";
      std::string input;
      std::cin >> input;
      int y;
      try
      {
        y = boost::lexical_cast<int> (input);
      }
      catch (boost::bad_lexical_cast const &)
      {    
        std::cout << "Error: input string was not valid" << std::endl;
        return -1;
      }
      nh.setParam("/binary_logger/log_repetition", y);
    }
    else if ((x < 0) || (x >= list_trjs.size()))
    {
      ROS_ERROR("out of bound");
      return -1;
    }
    else
    {
      ROS_INFO("selected %d) %s", x, list_trjs.at(x).c_str());
      trj_name = list_trjs.at(x).c_str();
      executed_trjs.push_back(trj_name);
    }
  }
  else
  {
    executed_trjs.push_back(trj_name);
  }

  move_group.startStateMonitor();
  moveit::core::RobotState trj_state = *move_group.getCurrentState();

  std::string log_name;
  if (!nh.getParam("/binary_logger/log_name", log_name))
  {
    log_name = "no_name";
  }

  bool save_time;
  if (!nh.getParam("/binary_logger/save_with_time", save_time))
  {
    save_time = true;
  }

  if (!nh.getParam("/binary_logger/log_repetition", repetions))
  {
    repetions = 1;
  }
  
  double extra_time;  //defined in seconds
  if (!nh.getParam("/binary_logger/extra_time", extra_time))
  {
    extra_time = 1;
  }

  bool rescale;
  if (!nh.getParam("/rescale", rescale))
  {
    rescale = true;
  }
  
  
  bool automatic_name=!log_name.compare("no_name");

  for (unsigned irep = 0; irep < repetions; irep++)
  {
    for (const std::string & current_trj_name : executed_trjs)
    {
      if (automatic_name)
      {
        log_name = current_trj_name;
      }

      ros::Time t0 = ros::Time::now();
      if (save_time == true)
      {
        log_name = log_name + "_" + std::to_string(t0.sec);
      }

      nh.setParam("/binary_logger/test_name", log_name + "_rep" + std::to_string(irep + 1));

      ros::ServiceClient start_log = nh.serviceClient<std_srvs::Empty> ("/start_log");
      ros::ServiceClient stop_log  = nh.serviceClient<std_srvs::Empty> ("/stop_log");

      trajectory_msgs::JointTrajectory trj;
      moveit_msgs::RobotTrajectory isp_trj;
      moveit_msgs::RobotTrajectory approach_trj;
      
      if (!trajectory_processing::getTrajectoryFromParam(nh, current_trj_name, trj))
      {
        ROS_ERROR("%s not found", current_trj_name.c_str());
        return 0;
      }
      // Next get the current set of joint values for the group.
      std::vector<double> initial_position = trj.points.begin()->positions;

      bool is_single_point = false;
      if (trj.points.size() < 2)
      {
        isp_trj.joint_trajectory = trj;
        is_single_point = true;
      }
      else
      {
        trj_state.setJointGroupPositions(group_name, initial_position);
        robot_trajectory::RobotTrajectory trajectory(move_group.getRobotModel(), group_name);
        trajectory.setRobotTrajectoryMsg(trj_state, trj);
        if (rescale)
        {
          trajectory_processing::IterativeSplineParameterization isp;
          isp.computeTimeStamps(trajectory);
          trajectory.getRobotTrajectoryMsg(isp_trj);
        }
        else 
          isp_trj.joint_trajectory=trj;
      }

      if (isp_trj.joint_trajectory.points.back().time_from_start > trj.points.back().time_from_start)
      {
        ROS_WARN("trajectory is scaled to respect joint limit");
      }

      move_group.setStartStateToCurrentState();
      move_group.setJointValueTarget(initial_position);
      
      robot_trajectory::RobotTrajectory trajectory(move_group.getRobotModel(), group_name);
      success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      if (!success)
      {
        ROS_ERROR("Planning failed");
        return 0;
      }
      
      trajectory.setRobotTrajectoryMsg(trj_state, my_plan.trajectory_.joint_trajectory);
      trajectory_processing::IterativeSplineParameterization isp;
      isp.computeTimeStamps(trajectory);
      trajectory.getRobotTrajectoryMsg(approach_trj);
      
      ROS_INFO("Move %s to initial position", group_name.c_str());
      std_srvs::Empty srv;
      moveit_msgs::ExecuteTrajectoryGoal goal;
      if (is_single_point)
      {
        start_log.call(srv);
        ros::Duration(3).sleep();
        goal.trajectory = approach_trj;
        ac.sendGoalAndWait(goal);
      }
      else
      {
        goal.trajectory = approach_trj;
        ac.sendGoalAndWait(goal);
        ros::Duration(3).sleep();

        start_log.call(srv);
        ros::Duration(1).sleep();
        ROS_INFO("Execute trajectory %s", current_trj_name.c_str());
        goal.trajectory = isp_trj;
        ac.sendGoalAndWait(goal);
      }
      
      ros::Duration(extra_time).sleep();
      stop_log.call(srv);
      ros::Duration(1).sleep();
    }
  }

  ros::shutdown();
  return 0;
}



