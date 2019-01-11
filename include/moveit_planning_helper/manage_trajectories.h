#ifndef MANAGE_TRJS_MOVEIT_HELPER___
#define MANAGE_TRJS_MOVEIT_HELPER___

#include <rosparam_utilities/rosparam_utilities.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit_planning_helper/iterative_spline_parameterization.h>
#include <ros/console.h>
#include <eigen_matrix_utils/eigen_matrix_utils.h>

namespace trajectory_processing
{
  inline bool sort_trajectory(const std::vector<std::string>& joint_names, const trajectory_msgs::JointTrajectory& trj, trajectory_msgs::JointTrajectory& sorted_trj)
  {
    const std::vector<std::string>& names=trj.joint_names;
    if (names.size()!=joint_names.size())
    {
      ROS_ERROR("Joint names dimensions are different");
      return false;
    }
    std::vector<int> order_idx(joint_names.size());
    
    
    
    for (unsigned int iOrder=0;iOrder<joint_names.size();iOrder++)
    {
      ROS_DEBUG("index %u, original trajectory %s, sorted trajectory %s",iOrder,names.at(iOrder).c_str(),joint_names.at(iOrder).c_str());
      if (names.at(iOrder).compare(joint_names.at(iOrder)))
      {
        for (unsigned int iNames=0;iNames<names.size();iNames++)
        {
          if (!joint_names.at(iOrder).compare(names.at(iNames)))
          { 
            order_idx.at(iOrder)=iNames;
            ROS_DEBUG("Joint %s (index %u) of original trajectory will be in position %u",names.at(iNames).c_str(),iOrder,iNames);
            break;
          }
          if (iNames==(names.size()-1))
          {
            ROS_ERROR("Joint %s missing",joint_names.at(iOrder).c_str());
            return false;
          }
        }
      }
      else 
      {
        order_idx.at(iOrder)=iOrder;
        ROS_DEBUG("Joint %s (index %u) of original trajectory will be in position %u",names.at(iOrder).c_str(),iOrder,iOrder);
      }
    }
    
    sorted_trj.joint_names=joint_names;
    sorted_trj.header=trj.header;
    
    for (const trajectory_msgs::JointTrajectoryPoint& pnt: trj.points)
    {
      sorted_trj.points.push_back(pnt);
      for (unsigned int iOrder=0;iOrder<joint_names.size();iOrder++)
      {
        sorted_trj.points.back().positions.at(iOrder)=pnt.positions.at(order_idx.at(iOrder));
        if (pnt.velocities.size()>0)
          sorted_trj.points.back().velocities.at(iOrder)=pnt.velocities.at(order_idx.at(iOrder));
        if (pnt.accelerations.size()>0)
          sorted_trj.points.back().accelerations.at(iOrder)=pnt.accelerations.at(order_idx.at(iOrder));
        if (pnt.effort.size()>0)
          sorted_trj.points.back().effort.at(iOrder)=pnt.effort.at(order_idx.at(iOrder));
      }
    }
    return true;
  }
  
  
  inline bool append_trajectories(trajectory_msgs::JointTrajectory& trj, const trajectory_msgs::JointTrajectory& trj_to_be_appended)
  {
    trajectory_msgs::JointTrajectory sorted_trj_to_be_appended;
    if (!sort_trajectory(trj.joint_names,trj_to_be_appended,sorted_trj_to_be_appended))
      return false;
    
    
    ros::Duration end_of_original_trj=trj.points.back().time_from_start;
    
    for (const trajectory_msgs::JointTrajectoryPoint& pnt: sorted_trj_to_be_appended.points)
    {
      trj.points.push_back(pnt);
      trj.points.back().time_from_start=trj.points.back().time_from_start+end_of_original_trj;
    }
    
    return true;
  }
  
  inline void removeDuplicates(trajectory_msgs::JointTrajectory& trj)
  {
    for (unsigned int iPnt=1;iPnt<trj.points.size();iPnt++)
    {
      if ((trj.points.at(iPnt).time_from_start.toSec()-trj.points.at(iPnt-1).time_from_start.toSec())>1e-6)
      {
        break;
      }
      
      for (unsigned int iax=0;iax<trj.points.at(iPnt).positions.size();iax++)
      {
        if (std::abs(trj.points.at(iPnt).positions.at(iax)-trj.points.at(iPnt-1).positions.at(iax))>1e-4)
        {
          break;
        }
      }
      
      ROS_FATAL("erasing point %u",iPnt);
      trj.points.erase(trj.points.begin()+iPnt);
    }
    return;
  };
  
  inline bool computeAccelerationVelocity(trajectory_msgs::JointTrajectory& trj)
  {
    if (trj.points.size()<2)
    {
      ROS_ERROR("trajectory should have at least 2 points");
      return false;
    }
    
    for (unsigned int iPnt=1;iPnt<trj.points.size();iPnt++)
    {
      trj.points.at(iPnt).velocities.resize(trj.points.at(iPnt).positions.size(),0);
      trj.points.at(iPnt).accelerations.resize(trj.points.at(iPnt).accelerations.size(),0);
    }
    //compute velocity  
    // first point
//     for  (unsigned int iAx=0;iAx<trj.points.at(0).positions.size();iAx++)
//     {
//       if ((trj.points.at(1).time_from_start.toSec()-trj.points.at(0).time_from_start.toSec())<=0)
//       {
//         ROS_ERROR("Time is not monotonically incresing");
//         return false;
//       }
//       trj.points.at(0).velocities.at(iAx)=(trj.points.at(1).positions.at(iAx)-trj.points.at(0).positions.at(iAx))/(trj.points.at(1).time_from_start.toSec()-trj.points.at(0).time_from_start.toSec());
//     }
    // from second point to last
    
    
    for (unsigned int iPnt=1;iPnt<(trj.points.size());iPnt++)
    {
      trj.points.at(iPnt).time_from_start=ros::Duration(trj.points.at(iPnt-1).time_from_start.toSec()+std::max(1.0e-5,trj.points.at(iPnt).time_from_start.toSec()-trj.points.at(iPnt-1).time_from_start.toSec()));
    }
      
    for (unsigned int iPnt=1;iPnt<(trj.points.size()-1);iPnt++)
      for  (unsigned int iAx=0;iAx<trj.points.at(iPnt).positions.size();iAx++)
        trj.points.at(iPnt).velocities.at(iAx)=(trj.points.at(iPnt+1).positions.at(iAx)-trj.points.at(iPnt-1).positions.at(iAx))/std::max(1e-4,trj.points.at(iPnt+1).time_from_start.toSec()-trj.points.at(iPnt-1).time_from_start.toSec());
     
    
      //compute accelerations
      // first point
      for  (unsigned int iAx=0;iAx<trj.points.at(0).accelerations.size();iAx++)
      {
        if ((trj.points.at(1).time_from_start.toSec()-trj.points.at(0).time_from_start.toSec())<0)
        {
          ROS_ERROR("Time is not monotonically incresing");
          return false;
        }
        
//         trj.points.at(0).accelerations.at(iAx)=0;
//         trj.points.at(0).accelerations.at(iAx)=0;
      }
      // from second point to last
      for (unsigned int iPnt=1;iPnt<(trj.points.size()-1);iPnt++)
        for  (unsigned int iAx=0;iAx<trj.points.at(iPnt).accelerations.size();iAx++)
          trj.points.at(iPnt).accelerations.at(iAx)=(trj.points.at(iPnt+1).velocities.at(iAx)-trj.points.at(iPnt-1).velocities.at(iAx))/std::max(1e-4,trj.points.at(iPnt+1).time_from_start.toSec()-trj.points.at(iPnt-1).time_from_start.toSec());
        
    return true;
  }

  
  inline bool getTrajectoryFromParam(const ros::NodeHandle& nh, const std::string& trj_name, trajectory_msgs::JointTrajectory& trj)
  {
    std::vector<std::vector<double>> positions;
    
    if (!rosparam_utilities::getParamMatrix(nh, trj_name+"/positions", positions))
    {
      ROS_ERROR("%s/positions does not exist", trj_name.c_str());
      return false;
    }
    unsigned int npnt = positions.size();
    
    std::vector<std::vector<double>> velocities;
    if (!rosparam_utilities::getParamMatrix(nh, trj_name+"/velocities", velocities))
    {
      ROS_DEBUG("%s/velocities does not exist", trj_name.c_str());
    }
    
    if (npnt == 0)
    {
      ROS_ERROR("%s/positions with no points", trj_name.c_str());
      return -1;
    }
    int dimension = positions.at(0).size();
    if (npnt == 0)
    {
      ROS_ERROR("%s/positions with no dimensions", trj_name.c_str());
      return false;
    }
    
    std::vector<double> time(positions.size(),0);
    if (!nh.getParam(trj_name+"/time_from_start", time))
    {
      ROS_DEBUG("%s/time_from_start does not exist", trj_name.c_str());
      
    }
    if (time.size() != npnt)
    {
      ROS_ERROR("%s/time_from_start has wrong dimensions", trj_name.c_str());
      return false;
    }
    trj.points.resize(npnt);
    
    trj.joint_names.resize(dimension);
    if (!nh.getParam(trj_name+"/joint_names",trj.joint_names))
    {
      for (int idx=0;idx<dimension;idx++)
        trj.joint_names.at(idx)= "joint_"+std::to_string(idx);
    }
    
    for (unsigned int iPnt = 0;iPnt<npnt;iPnt++)
    {
      trj.points.at(iPnt).positions.resize(dimension);
      trj.points.at(iPnt).positions = positions.at(iPnt);
      if (velocities.size()>0)
      {
        trj.points.at(iPnt).velocities.resize(dimension);
        trj.points.at(iPnt).velocities = velocities.at(iPnt);
      }
      trj.points.at(iPnt).accelerations.resize(dimension,0);
      trj.points.at(iPnt).effort.resize(dimension,0);
      
      trj.points.at(iPnt).time_from_start = ros::Duration(time.at(iPnt));
    }
    return true;
  }
  
  inline bool setTrajectoryToParam( ros::NodeHandle& nh, const std::string& trj_name, const trajectory_msgs::JointTrajectory& trj)
  {
    if (trj.points.size()==0)
    {
      ROS_WARN("Trajectory with no points");
      return false;
    }
    
    std::vector<std::vector<double>> positions(trj.points.size());
    std::vector<std::vector<double>> velocities(trj.points.size());
    std::vector<std::vector<double>> accelerations(trj.points.size());
    std::vector<std::vector<double>> effort(trj.points.size());
    std::vector<double> time(trj.points.size());
    
    for (unsigned int iPnt=0; iPnt<trj.points.size(); iPnt++)
    {
      positions.at(iPnt) = trj.points.at(iPnt).positions;
      velocities.at(iPnt) = trj.points.at(iPnt).velocities;
      accelerations.at(iPnt) = trj.points.at(iPnt).accelerations;
      effort.at(iPnt) = trj.points.at(iPnt).effort;
      time.at(iPnt) = trj.points.at(iPnt).time_from_start.toSec();
    }
    
    nh.setParam(trj_name+"/joint_names",trj.joint_names);
    nh.setParam(trj_name+"/time_from_start",time);
    
    
    if (!rosparam_utilities::setParam<double>(nh,trj_name+"/positions",positions))
      return false;
    if (!rosparam_utilities::setParam(nh,trj_name+"/velocities",velocities))
      return false;
    if (!rosparam_utilities::setParam(nh,trj_name+"/accelerations",accelerations))
      return false;
    if (!rosparam_utilities::setParam(nh,trj_name+"/effort",effort))
      return false;
    
    return true;
  }
  
  inline trajectory_msgs::JointTrajectory createDenseTrajectory(const trajectory_msgs::JointTrajectory& trj, double sampling_period);
  
  bool checkCollisionBetweenTrajectories(const robot_trajectory::RobotTrajectory& trj1, const robot_trajectory::RobotTrajectory& trj2, const std::__cxx11::string& joined_group_name);
}


#endif
