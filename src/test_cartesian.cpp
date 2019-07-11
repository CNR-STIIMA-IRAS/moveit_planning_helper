
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


bool is_received = false;
bool is_executing = false;
geometry_msgs::Pose target_pose;
ros::CallbackQueue queue;

void provaPoseCallback(const geometry_msgs::PoseConstPtr& msg)
{
  if (!is_executing)
  {
    target_pose = *msg;
    ROS_INFO("geometry_msgs/Pose Message received");
  }

  is_received = true;
  ROS_WARN_THROTTLE(0.5, "Robot is moving, unable to receive new target pose");

}



int main(int argc, char **argv)
{

  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  std::shared_ptr<ros_helper::SubscriptionNotifier<geometry_msgs::Pose>> m_scaling_in_sub;
  m_scaling_in_sub.reset(new ros_helper::SubscriptionNotifier<geometry_msgs::Pose>(node_handle,"/prova_pose",1));
  m_scaling_in_sub->setAdvancedCallback(&provaPoseCallback);

  //  moveit::planning_interface::MoveGroupInterface group("ur10");
  moveit::planning_interface::MoveGroupInterface group("ur5");
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  group.setStartStateToCurrentState();
  group.setPlanningTime(10.0);

  std::vector<geometry_msgs::Pose> waypoints;


  //Print actual cartesian pose
  target_pose = group.getCurrentPose().pose;
  ROS_INFO("Current pose:");
  ROS_INFO("Position x: %.2f", target_pose.position.x);
  ROS_INFO("Position y: %.2f", target_pose.position.y);
  ROS_INFO("Position z: %.2f", target_pose.position.z);
  ROS_INFO("Orientation x: %.2f", target_pose.orientation.x);
  ROS_INFO("Orientation y: %.2f", target_pose.orientation.y);
  ROS_INFO("Orientation z: %.2f", target_pose.orientation.z);
  ROS_INFO("Orientation w: %.2f", target_pose.orientation.w);




  while(ros::ok())
  {
    if(is_received)
    {
      is_executing = true;
      waypoints.clear();
      waypoints.push_back(target_pose);

      moveit_msgs::RobotTrajectory trajectory_msg;


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


      if(group.execute(plan))
        ROS_INFO("Trajectory executed");
      else
      {
        ROS_INFO("Error during trajectory execution");
        return 0;
      }


      std::cout << "-------------" << std::endl;
      ROS_INFO("Current pose:");
      ROS_INFO("Position x: %.2f", target_pose.position.x);
      ROS_INFO("Position y: %.2f", target_pose.position.y);
      ROS_INFO("Position z: %.2f", target_pose.position.z);
      ROS_INFO("Orientation x: %.2f", target_pose.orientation.x);
      ROS_INFO("Orientation y: %.2f", target_pose.orientation.y);
      ROS_INFO("Orientation z: %.2f", target_pose.orientation.z);
      ROS_INFO("Orientation w: %.2f", target_pose.orientation.w);

      is_received = false;
      is_executing = false;

    }

  }
  return 0;
}



