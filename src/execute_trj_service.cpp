
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit_msgs/ExecuteTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <boost/lexical_cast.hpp>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
#include <moveit_planning_helper/manage_trajectories.h>

#include <moveit_planning_helper/ExecuteTrajectoryFromParamAction.h>
#include <thread>

std::shared_ptr<actionlib::SimpleActionServer<moveit_planning_helper::ExecuteTrajectoryFromParamAction>> as;
ros::Publisher display_trj_pub;

void ExecuteTrajectoryFromParamAction( const moveit_planning_helper::ExecuteTrajectoryFromParamGoalConstPtr& goal )
{
  ROS_INFO("new goal");
  moveit_planning_helper::ExecuteTrajectoryFromParamResult result;
  result.ok=false;
  ros::NodeHandle nh;
  actionlib::SimpleActionClient<moveit_msgs::ExecuteTrajectoryAction> ac(nh, "/execute_trajectory");
  moveit_msgs::DisplayTrajectory display_trj_msg;
  
  bool simulate=goal->simulation;
  std::string group_name=goal->group_name;
  
  
 
  std::vector<std::string> executed_trjs=goal->trajectory_names;
  
  if (!simulate)
  {
    ROS_FATAL("execute");
    if (!ac.waitForServer(ros::Duration(10)))
    {
      ROS_ERROR("NO /execute_trajectory server");
      as->setAborted(result);
      return ;
    }
  }
  else
    ROS_FATAL("SIMULATED");
  
  
  
  moveit::planning_interface::MoveGroupInterface move_group(group_name);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success;
  
  
  if (executed_trjs.size()==0)
  {
    if (!nh.getParam("list_of_trajectories", executed_trjs))
    {
      ROS_ERROR("NO list_of_trajectories specified");
      as->setAborted(result);
      return ;
    }
  }
  
  int repetions;
  if (!nh.getParam("/binary_logger/log_repetition", repetions))
  {
    repetions = 1;
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
      std::string binary_log_name;
      if (automatic_name)
        binary_log_name = current_trj_name;
      else 
        binary_log_name=log_name;
      
      ros::Time t0 = ros::Time::now();
      if (save_time == true)
        binary_log_name = binary_log_name + "_" + std::to_string(t0.sec);
      
      nh.setParam("/binary_logger/test_name", binary_log_name + "_rep" + std::to_string(irep + 1));
      
      ros::ServiceClient start_log = nh.serviceClient<std_srvs::Empty> ("/start_log");
      ros::ServiceClient stop_log  = nh.serviceClient<std_srvs::Empty> ("/stop_log");
      
      trajectory_msgs::JointTrajectory trj;
      moveit_msgs::RobotTrajectory isp_trj;
      moveit_msgs::RobotTrajectory approach_trj;
      
      if (!trajectory_processing::getTrajectoryFromParam(nh, current_trj_name, trj))
      {
        ROS_ERROR("%s not found", current_trj_name.c_str());
        return;
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
      

      move_group.startStateMonitor(2);
      move_group.setStartState(*move_group.getCurrentState());
      //move_group.setStartStateToCurrentState();
      move_group.setJointValueTarget(initial_position);
      
      robot_trajectory::RobotTrajectory trajectory(move_group.getRobotModel(), group_name);
      success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      if (!success)
      {
        ROS_ERROR("Planning failed");
        return;
      }

      
      trajectory.setRobotTrajectoryMsg(trj_state, my_plan.trajectory_.joint_trajectory);
      trajectory_processing::IterativeSplineParameterization isp;
      isp.computeTimeStamps(trajectory);
      trajectory.getRobotTrajectoryMsg(approach_trj);
      
      if (!simulate)
        ROS_INFO("Move %s to initial position", group_name.c_str());
      std_srvs::Empty srv;
      moveit_msgs::ExecuteTrajectoryGoal goal;
      if (is_single_point && (!simulate))
      {
        start_log.call(srv);
        ros::Duration(3).sleep();
        goal.trajectory = approach_trj;
        ac.sendGoalAndWait(goal);
        
      }
      else
      {
        if (!simulate)
        {
          ROS_INFO("QUI");
          goal.trajectory = approach_trj;
          ac.sendGoalAndWait(goal);
          ros::Duration(3).sleep();
          
          start_log.call(srv);
          ros::Duration(1).sleep();
          ROS_INFO("Execute trajectory %s", current_trj_name.c_str());
          goal.trajectory = isp_trj;
          ac.sendGoalAndWait(goal);
        }
        else 
        {
          ROS_INFO("QUI!!");
          display_trj_msg.trajectory.push_back(approach_trj);
          display_trj_pub.publish(display_trj_msg);
          approach_trj.joint_trajectory.points.back().time_from_start.sleep();
          ROS_INFO("QUI!!");
          
          display_trj_msg.trajectory.at(0)=isp_trj;
          display_trj_pub.publish(display_trj_msg);
          isp_trj.joint_trajectory.points.back().time_from_start.sleep();
          ROS_INFO("QUI!!");
          
        }
      }
      if (!simulate)
      {
        ros::Duration(extra_time).sleep();
        stop_log.call(srv);
        ros::Duration(1).sleep();
      }
    }
  }
  
  result.ok=true;
  as->setSucceeded(result);
  return ;
};

int main(int argc, char **argv)
{
 
  ros::init(argc, argv, "execute_save_trajectory");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(4);
  spinner.start();
  as.reset(new actionlib::SimpleActionServer<moveit_planning_helper::ExecuteTrajectoryFromParamAction>(nh,"/execute_trajectories_from_param",ExecuteTrajectoryFromParamAction,false ));
  as->start();                                                                                                   
  
  display_trj_pub=nh.advertise<moveit_msgs::DisplayTrajectory>("rosdyn/simulated_trajectory",1,true);
  
  moveit_msgs::DisplayTrajectory display_trj_msg;
  display_trj_pub.publish(display_trj_msg);
  
  
  while (ros::ok())
    ros::Duration(.01).sleep();
 
  return 0;
}



