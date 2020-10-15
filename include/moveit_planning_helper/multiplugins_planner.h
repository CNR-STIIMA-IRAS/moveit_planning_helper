#ifndef multiplugins_planner_201808271745
#define multiplugins_planner_201808271745
#include <ros/ros.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit_planning_helper/iterative_spline_parameterization.h>
#include <moveit_planning_helper/manage_trajectories.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <pluginlib/class_loader.hpp>
#include <boost/smart_ptr.hpp>
#include <moveit_planning_helper/iterative_spline_parameterization.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit/move_group_interface/move_group_interface.h>

namespace moveit_helper
{
  class MultipluginsPlanner 
  {
  protected:
    robot_model::RobotModelPtr m_robot_model;
    planning_scene::PlanningScenePtr m_planning_scene;
    std::vector<planning_interface::PlannerManagerPtr> m_planner_instance;
    robot_state::JointModelGroup* m_joint_model_group;
    trajectory_processing::IterativeSplineParameterization m_isp;
    moveit::planning_interface::MoveGroupInterfacePtr m_move_group;
  public:
    MultipluginsPlanner(const std::string& robot_description, 
                        const std::string& group_name, 
                        const std::vector< std::string >& plugins_name, 
                        const ros::NodeHandle& nh);
    
  bool planJoint(const std::vector< double >& destination, trajectory_msgs::JointTrajectory& trj);
    
  };
}

#endif