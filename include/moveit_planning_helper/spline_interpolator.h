#ifndef SPLINE_INTERPOLATOR_201810021818
#define SPLINE_INTERPOLATOR_201810021818

#include <trajectory_msgs/JointTrajectory.h>
#include <ros/console.h>
#include <moveit_msgs/RobotTrajectory.h>

namespace trajectory_processing
{
  
  class SplineInterpolator
  {
  protected:
    trajectory_msgs::JointTrajectoryPtr m_trj;
    unsigned int m_order;
  public:
    SplineInterpolator();
    bool setTrajectory(const trajectory_msgs::JointTrajectoryPtr& trj);
    bool setTrajectory(const trajectory_msgs::JointTrajectory& trj);
    bool setTrajectory(const moveit_msgs::RobotTrajectoryPtr &trj);
    bool setTrajectory(const moveit_msgs::RobotTrajectory &trj);
    trajectory_msgs::JointTrajectoryPtr getTrajectoryMsg();


    /* setSplineOrder:
     * order = 0 positions are continuous
     * order = 1 velocities are continuous
     * order = 2 accelerations are continuous
     * order = 3 jerks are continuous, supposed zero at the waypoints
     * order = 4 snaps are continuous, supposed zero at the waypoints
     */
    void setSplineOrder(const unsigned int& order);
    bool interpolate(const ros::Duration& time, trajectory_msgs::JointTrajectoryPoint& pnt, const double& scaling=1.0);
    bool resampleTrajectory(const double delta_time, trajectory_msgs::JointTrajectory& trj);
    bool resampleTrajectory(const double delta_time, moveit_msgs::RobotTrajectory& trj);

    ros::Duration trjTime();
  };
  
}













#endif
