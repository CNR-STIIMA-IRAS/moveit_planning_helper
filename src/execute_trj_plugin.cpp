#include <ros/ros.h>
#include <pluginlib/class_loader.h>

#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit_planning_helper/manage_trajectories.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_planning_helper/iterative_spline_parameterization.h>

#include <actionlib/client/simple_action_client.h>
#include <moveit_msgs/ExecuteTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <std_msgs/Float64.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Empty.h>

#include <boost/lexical_cast.hpp>


#define ANSI_COLOR_RED     "\x1b[31m"
#define ANSI_COLOR_GREEN   "\x1b[32m"
#define ANSI_COLOR_YELLOW  "\x1b[33m"
#define ANSI_COLOR_BLUE    "\x1b[34m"
#define ANSI_COLOR_MAGENTA "\x1b[35m"
#define ANSI_COLOR_CYAN    "\x1b[36m"
#define ANSI_COLOR_RESET   "\x1b[0m"
#define ANSI_COLOR_BOLDWHITE  "\033[1m\033[37m"

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
  if (!nh.getParam("move_group",group_name))
  {
    ROS_ERROR("move_group name is undefined");
    return -1;
  }
  ros::NodeHandle mg_nh(group_name);
  
  bool use_execution_trajectory;
  if (!nh.getParam("use_execution_trajectory",use_execution_trajectory ))
    use_execution_trajectory=true;
  
  actionlib::SimpleActionClient<moveit_msgs::ExecuteTrajectoryAction> extrj_ac(nh,"/execute_trajectory");
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> jnttrj_ac(nh,"/"+group_name+"/follow_joint_trajectory");
  
  if (use_execution_trajectory)
  {
    if (!extrj_ac.waitForServer(ros::Duration(10)))
    {
      ROS_ERROR("NO /execute_trajectory server");
      return 0;
    }
  }
  else    
  {
    if (!jnttrj_ac.waitForServer(ros::Duration(10)))
    {
      ROS_ERROR("NO /%s/follow_joint_trajectory server",group_name.c_str());
      return 0;
    }
  }
  
  std::string robot_description = "robot_description";
  if (!nh.getParam("robot_description_name",robot_description))
    robot_description = "robot_description";
  
  robot_model_loader::RobotModelLoader robot_model_loader(robot_description);
  robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();
  if (!robot_model)
  {
    ROS_ERROR("Model is not loaded");
    return -1;
  }
  
  
  
  
  moveit::planning_interface::MoveGroupInterface move_group( group_name );
  planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));
  
  boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;
  std::vector<planning_interface::PlannerManagerPtr> planner_instance;
  std::vector<std::string> planner_plugin_name_vector;
  
  if (!mg_nh.getParam("planning_plugin", planner_plugin_name_vector))
  {
    ROS_ERROR_STREAM("Could not find planner plugin name");
    return 0;
  }
  try
  {
    planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>(
      "moveit_core", "planning_interface::PlannerManager"));
  }
  catch (pluginlib::PluginlibException& ex)
  {
    ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
  }
  
  for (const std::string& planner_plugin_name: planner_plugin_name_vector)
  {
    try
    {
      planning_interface::PlannerManagerPtr new_planner_instance;
      new_planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
      if (!new_planner_instance->initialize(robot_model, mg_nh.getNamespace()))
        ROS_FATAL_STREAM("Could not initialize planner instance");
      ROS_INFO_STREAM("Using planning interface '" << new_planner_instance->getDescription() << "'");
      planner_instance.push_back(new_planner_instance);
    }
    catch (pluginlib::PluginlibException& ex)
    {
      const std::vector<std::string>& classes = planner_plugin_loader->getDeclaredClasses();
      std::stringstream ss;
      for (std::size_t i = 0; i < classes.size(); ++i)
        ss << classes[i] << " ";
      ROS_ERROR_STREAM("Exception while loading planner '" << planner_plugin_name << "': " << ex.what() << std::endl
      << "Available plugins: " << ss.str());
    }
  }
  planning_interface::MotionPlanRequest req;
  planning_interface::MotionPlanResponse res;
  req.allowed_planning_time=5;
  
  robot_state::RobotState goal_state(robot_model);
  const robot_state::JointModelGroup* joint_model_group = goal_state.getJointModelGroup(move_group.getName());
  if (!joint_model_group)
  {
    ROS_ERROR("Void joint_model_group");
    return 0;
  }
  
  std::string execute_trj_name;
  if (!nh.getParam("trj_name",execute_trj_name))
  {
    ROS_ERROR("NO trj_name specified");
    return -1;
  }
  
  
  while (ros::ok())
  {
    
    bool recompute_timestamps;
    double speed_scaling;
    double acc_scaling;
    
    if (!nh.getParam("recompute_timestamps",recompute_timestamps))
    {
      recompute_timestamps=true;
      nh.setParam("recompute_timestamps",recompute_timestamps);
    }
    if (!nh.getParam("speed_scaling",speed_scaling))
    {
      speed_scaling=0.4;
      nh.setParam("speed_scaling",speed_scaling);
    }
    if (!nh.getParam("acc_scaling",acc_scaling))
    {
      acc_scaling=0.4;
      nh.setParam("acc_scaling",acc_scaling);
    }
    
    
    std::vector<std::string> executed_trjs;
    
    if (!execute_trj_name.compare("load"))
    {
      std::vector<std::string> list_trjs;
      if (!nh.getParam("/list_of_trajectories",list_trjs))
      {
        ROS_ERROR("NO list_of_trajectories specified");
        return -1;
      }
      printf(ANSI_COLOR_YELLOW "select trajectory:\n" ANSI_COLOR_RESET);
      printf(ANSI_COLOR_GREEN "time stamps:    %s \n" ANSI_COLOR_RESET,(recompute_timestamps? "online computed": "original"));
      printf(ANSI_COLOR_CYAN "speed_scaling = %.3f\n" ANSI_COLOR_RESET,speed_scaling);
      printf(ANSI_COLOR_GREEN "acc_scaling   = %.3f\n" ANSI_COLOR_RESET,acc_scaling);
      printf(ANSI_COLOR_CYAN "-10) execute all the trajectories\n" ANSI_COLOR_RESET);
      printf(ANSI_COLOR_GREEN " -9) change speed_scaling\n" ANSI_COLOR_RESET);
      printf(ANSI_COLOR_CYAN " -8) change acc_scaling\n" ANSI_COLOR_RESET);
      printf(ANSI_COLOR_GREEN " -7) turn on/off timestamps online computation\n" ANSI_COLOR_RESET);
      printf(ANSI_COLOR_CYAN " -1) shutdown\n" ANSI_COLOR_RESET);
      for (unsigned int idx=0;idx<list_trjs.size();idx++)
        printf(ANSI_COLOR_YELLOW "%3d)" ANSI_COLOR_BOLDWHITE " %s\n" ANSI_COLOR_RESET,idx,list_trjs.at(idx).c_str());
      
      std::cout <<"enter number: ";
      std::string input;
      std::cin>>input;
      
      int x;
      double  y;
      
      try 
      {
        x = boost::lexical_cast<int>( input );
      } 
      catch( boost::bad_lexical_cast const& ) 
      {
        std::cout << "Error: input string was not valid" << std::endl;
        continue;
      }
      if (x==-1)
      {
        return 0;
      }
      else if (x==-10)
      {
        ROS_INFO("execute all the trajectories");
        executed_trjs=list_trjs;
      }
      else if (x==-9)
      {
        ROS_INFO("enter new speed_scaling [0,1]");
        std::cin>>input;
        try 
        {
          y = boost::lexical_cast<double>( input );
        } 
        catch( boost::bad_lexical_cast const& ) 
        {
          std::cout << "Error: input string was not valid" << std::endl;
          continue;
        }
        if ((y<0)||(y>1))
        {
          ROS_ERROR("out of bound");
          continue;
        }
        nh.setParam("speed_scaling",y);
        continue;
        
      }
      else if (x==-8)
      {
        ROS_INFO("enter new acc_scaling [0,1]");
        std::cin>>input;
        try 
        {
          y = boost::lexical_cast<double>( input );
        } 
        catch( boost::bad_lexical_cast const& ) 
        {
          std::cout << "Error: input string was not valid" << std::endl;
          continue;
        }
        if ((y<0)||(y>1))
        {
          ROS_ERROR("out of bound");
          continue;
        }
        nh.setParam("acc_scaling",y);
        continue;
        
      }
      else if (x==-7)
      {
        ROS_INFO("enter 0 to use original, other to recompute timestamps");
        std::cin>>input;
        try 
        {
          x = boost::lexical_cast<double>( input );
        } 
        catch( boost::bad_lexical_cast const& ) 
        {
          std::cout << "Error: input string was not valid" << std::endl;
          continue;
        }
        if (x==0)
          nh.setParam("recompute_timestamps",false);
        else 
          nh.setParam("recompute_timestamps",true);
        continue;
        
      }
      else if ((x<0) || (x>=list_trjs.size()))
      {
        ROS_ERROR("out of bound");
        continue;
      }
      else 
      {
        ROS_INFO("selected %d) %s",x,list_trjs.at(x).c_str());
        executed_trjs.push_back(list_trjs.at(x));
      }
    }
    else 
      executed_trjs.push_back(execute_trj_name);
    
    planning_interface::PlanningContextPtr  context;
    
    for (const std::string& trj_name: executed_trjs)
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
      
      
      bool joint_names_error=false;
      if (trj.joint_names.size()!=joint_model_group->getActiveJointModelNames().size())
      {
        joint_names_error=true;
      }
      else 
      {
        for (unsigned int iax=0;iax<trj.joint_names.size();iax++)
          if (trj.joint_names.at(iax).compare(joint_model_group->getActiveJointModelNames().at(iax)))
          {
            ROS_ERROR("joint %u has wrong name (%s instead of %s)",iax,trj.joint_names.at(iax).c_str(),joint_model_group->getActiveJointModelNames().at(iax).c_str());
            joint_names_error=true;
          }
      }
          
      if (joint_names_error)
      {
        ROS_ERROR("Joint names are not consistent");
        ROS_ERROR("Trajectory joint names are:");
        for (const std::string& name: trj.joint_names)
          ROS_ERROR("-%s", name.c_str());
        
        
        ROS_ERROR("Joint model names are:");
        for (const std::string& name: joint_model_group->getActiveJointModelNames())
          ROS_ERROR("-%s", name.c_str());
        
        return -1;
      }
      // Next get the current set of joint values for the group.
      ROS_INFO("plan path to initial point");
      goal_state.setJointGroupPositions(joint_model_group, trj.points.begin()->positions);
      moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group);
      req.goal_constraints.clear();
      req.goal_constraints.push_back(joint_goal);
      req.group_name=joint_model_group->getName();
      
      
      moveit::core::robotStateToRobotStateMsg(*move_group.getCurrentState(),req.start_state);
      
      bool planner_success=false;
      for (planning_interface::PlannerManagerPtr& act_planner_instance: planner_instance)
      {
        ROS_INFO("try with planner %s",act_planner_instance->getDescription().c_str());
        context = act_planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
        context->solve(res);
        if (res.error_code_.val != res.error_code_.SUCCESS)
        {
          ROS_WARN_STREAM("planning interface '" << act_planner_instance->getDescription() << "' is not able to plan");
          continue;
        }
        
        planner_success=true;
        break;
      }
      if (!planner_success)
      {
        ROS_ERROR("Could not compute plan to initial position successfully");
        continue;
      }  
      
      trajectory_processing::IterativeSplineParameterization isp;
      isp.computeTimeStamps(*res.trajectory_,speed_scaling,acc_scaling);
      
      
      moveit_msgs::ExecuteTrajectoryGoal goal;
      res.trajectory_->getRobotTrajectoryMsg(goal.trajectory);
      
      ROS_INFO("Move %s to initial position",group_name.c_str());
      
      if (use_execution_trajectory)
        extrj_ac.sendGoalAndWait(goal);
      else 
      {
        control_msgs::FollowJointTrajectoryGoal jnttrj_goal;
        jnttrj_goal.trajectory=goal.trajectory.joint_trajectory;
        jnttrj_ac.sendGoalAndWait(jnttrj_goal);
      }
      
      
      robot_trajectory::RobotTrajectoryPtr trj_ptr(new robot_trajectory::RobotTrajectory(robot_model,group_name));
      trj_ptr->setRobotTrajectoryMsg(goal_state,trj);
      if (recompute_timestamps)
        isp.computeTimeStamps(*trj_ptr,speed_scaling,acc_scaling);
      
      
      ros::Duration(3).sleep();
      
      std_srvs::Empty srv;
      start_log.call(srv);
      ros::Duration(1).sleep();
      trj_ptr->getRobotTrajectoryMsg(goal.trajectory);
      
      ROS_INFO("Execute trajectory %s",trj_name.c_str());
      if (use_execution_trajectory)
        extrj_ac.sendGoalAndWait(goal);
      else 
      {
        control_msgs::FollowJointTrajectoryGoal jnttrj_goal;
        jnttrj_goal.trajectory=goal.trajectory.joint_trajectory;
        jnttrj_ac.sendGoalAndWait(jnttrj_goal);
      }
      
      ros::Duration(1).sleep();
      stop_log.call(srv);
      ros::Duration(1).sleep();
    }
  }
  
  ros::shutdown();
  return 0;
}
