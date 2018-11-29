#ifndef SPLINE_INTERPOLATOR_201810021818
#define SPLINE_INTERPOLATOR_201810021818

#include <trajectory_msgs/JointTrajectory.h>
#include <ros/console.h>
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
    void setSplineOrder(const unsigned int& order);
    bool interpolate(const ros::Duration& time, trajectory_msgs::JointTrajectoryPoint& pnt, const double& scaling=1.0);
    bool resampleTrajectory(const double delta_time, trajectory_msgs::JointTrajectory& trj);
    ros::Duration trjTime();
  };
  
}













#endif