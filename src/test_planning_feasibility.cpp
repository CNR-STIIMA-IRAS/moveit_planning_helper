
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

#include <subscription_notifier/subscription_notifier.h>


#include <moveit_planning_helper/manage_trajectories.h>

int main(int argc, char **argv)
{

  ros::init(argc, argv, "test_planning_feasibility");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(4);
  spinner.start();

  std::string group_name = "manipulator";
  moveit::planning_interface::MoveGroupInterface group(group_name);
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr       kinematic_model = robot_model_loader.getModel();
  planning_scene::PlanningScenePtr planning_scene = std::make_shared<planning_scene::PlanningScene>(kinematic_model);
  robot_state::RobotState start_state = planning_scene->getCurrentState();
  robot_state::RobotState target_state = planning_scene->getCurrentState();


  float planning_trials=0;
  float planning_success=0;

  float num_goal_points=10;

  ros::Rate lp(10);
  while(ros::ok())
  {
    unsigned int itrial=0;
    for (itrial=0;itrial<100;itrial++)
    {
      start_state.setToRandomPositions();
      start_state.update();
      start_state.updateCollisionBodyTransforms();
      if (!start_state.satisfiesBounds())
      {
        ROS_DEBUG("Target state is out of bound");
        continue;
      }
      if (!planning_scene->isStateValid(start_state,group_name))
      {
        ROS_DEBUG("Target state is in collision");
        continue;
      }
      break;
    }
    if (itrial>=100)
      continue;
    group.setStartState(start_state);

    float batch_planning_trials=0;
    float batch_planning_success=0;

    for (float batch=0;batch<num_goal_points;batch++)
    {

      for (itrial=0;itrial<100;itrial++)
      {
        target_state.setToRandomPositions();
        target_state.update();
        target_state.updateCollisionBodyTransforms();
        if (!target_state.satisfiesBounds())
        {
          ROS_DEBUG("Target state is out of bound");
          continue;
        }
        if (!planning_scene->isStateValid(target_state,group_name))
        {
          ROS_DEBUG("Target state is in collision");
          continue;
        }
        break;
      }
      if (itrial>=100)
        continue;
      group.setJointValueTarget(target_state);

      moveit::planning_interface::MoveGroupInterface::Plan plan;

      bool success = (group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      ROS_INFO_STREAM("Planning from:");
      Eigen::VectorXd q;
      start_state.copyJointGroupPositions(group_name,q);
      ROS_INFO_STREAM("Start  state\n"<<q.transpose());
      target_state.copyJointGroupPositions(group_name,q);
      ROS_INFO_STREAM("Target state\n"<<q.transpose());

      planning_trials++;
      batch_planning_trials++;

      if (!success)
        ROS_ERROR("Planning failed");
      else
      {
        planning_success++;
        batch_planning_success++;
      }
      ROS_INFO("trajectory length=%f",trajectory_processing::computeTrajectoryLength(plan.trajectory_.joint_trajectory));

      ROS_INFO("Success rate =%f %%, batch=%f %%,  trials=%d, batch trial=%d",planning_success/planning_trials*100,
               batch_planning_success/batch_planning_trials*100,
               (int)planning_trials,(int)batch+1);
      ros::Duration(0.5).sleep();
    }
  }
  return 0;
}



