#include <moveit_planning_helper/multiplugins_planner.h>
#include <moveit/robot_state/conversions.h>


namespace moveit_helper 
{

MultipluginsPlanner::MultipluginsPlanner(const std::string& robot_description, const std::string& group_name, const std::vector< std::string >& plugins_name, const ros::NodeHandle& nh)
{
  m_move_group.reset(new moveit::planning_interface::MoveGroupInterface(group_name) );
  
  robot_model_loader::RobotModelLoader robot_model_loader(robot_description);
  m_robot_model= robot_model_loader.getModel();
  
  m_planning_scene.reset(new planning_scene::PlanningScene(m_robot_model));
  boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;
  
  m_joint_model_group = m_robot_model->getJointModelGroup(group_name);
   
  try
  {
    planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>(
      "moveit_core", "planning_interface::PlannerManager"));
  }
  catch (pluginlib::PluginlibException& ex)
  {
    ROS_DEBUG_STREAM("Exception while creating planning plugin loader " << ex.what());
  }
  
  for (const std::string& planner_plugin_name: plugins_name)
  {
    try
    {
      planning_interface::PlannerManagerPtr new_planner_instance;
      new_planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
      if (!new_planner_instance->initialize(m_robot_model, nh.getNamespace()))
        ROS_DEBUG_STREAM("Could not initialize planner instance");
      ROS_INFO_STREAM("Using planning interface '" << new_planner_instance->getDescription() << "'");
      m_planner_instance.push_back(new_planner_instance);
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
  
  
}
  
bool MultipluginsPlanner::planJoint(const std::vector< double >& destination, trajectory_msgs::JointTrajectory& trj)
{
  robot_state::RobotState goal_state(m_robot_model);
  
  goal_state.setJointGroupPositions(m_joint_model_group, destination);
  moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(goal_state, m_joint_model_group);
  planning_interface::MotionPlanRequest req;
  planning_interface::MotionPlanResponse res;
  
  
  moveit::core::robotStateToRobotStateMsg(*m_move_group->getCurrentState(),req.start_state);
  req.allowed_planning_time=5;
  req.goal_constraints.clear();
  req.goal_constraints.push_back(joint_goal);
  req.group_name=m_joint_model_group->getName();
  
  bool planner_success=false;
  planning_interface::PlanningContextPtr  context;
  
  for (planning_interface::PlannerManagerPtr& act_planner_instance: m_planner_instance)
  {
    
    ROS_DEBUG("try with planner %s",act_planner_instance->getDescription().c_str());
    context = act_planner_instance->getPlanningContext(m_planning_scene, req, res.error_code_);
    context->solve(res);
    
    if (res.error_code_.val != res.error_code_.SUCCESS)
    {
      ROS_DEBUG_STREAM("planning interface '" << act_planner_instance->getDescription() << "' is not able to plan");
      continue;
    }
    ROS_DEBUG_STREAM("planning interface '" << act_planner_instance->getDescription() << "' planned successfully");
    planner_success=true;
    break;
  }
  if (!planner_success)
    return false;
  
  
  m_isp.computeTimeStamps(*res.trajectory_);
  moveit_msgs::RobotTrajectory resulting_trajectory;
  res.trajectory_->getRobotTrajectoryMsg(resulting_trajectory);
  trj=resulting_trajectory.joint_trajectory;
  
  
  
}


}