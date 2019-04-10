#ifndef __itia_planning_scene_conversion_h__
#define __itia_planning_scene_conversion_h__

#include <iostream>
#include <fstream>
#include <vector>

#include <ros/ros.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>
#include <geometric_shapes/shape_operations.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

namespace moveit_planning_helper
{
  
/**
 * @fn robotStateToPoseMsg
 */
inline geometry_msgs::Pose robotStateToPoseMsg ( const robot_state::RobotState&  rstate, const std::string& last_link )
{
  Eigen::Affine3d ee = rstate.getGlobalLinkTransform(last_link);
  geometry_msgs::Pose ee_msgs;
  tf::poseEigenToMsg(ee, ee_msgs);
  return ee_msgs;
}


inline void stlToMeshMsg ( const std::string& stlfilename, shape_msgs::Mesh& mesh )
{
  shapes::Mesh*                   m = shapes::createMeshFromResource ( stlfilename );
//shape_msgs::Mesh*     collision_object_mesh = new (shape_msgs::Mesh);
  shapes::ShapeMsg                collision_object_mesh_msg;
  shapes::constructMsgFromShape ( m, collision_object_mesh_msg );
  mesh = boost::get<shape_msgs::Mesh>( collision_object_mesh_msg );
  delete m;
  return;
}

inline void meshToCollisionObjectMsg ( const shape_msgs::Mesh& collision_object_mesh
                                     , const std::string& frame_id
                                     , const std::string& id
                                     , const geometry_msgs::Pose& pose
                                     , moveit_msgs::CollisionObject& co_msg )
{

  co_msg.header.frame_id = frame_id;
  co_msg.id              = id;
  co_msg.meshes.resize ( 1 );
  co_msg.mesh_poses.resize ( 1 );

  co_msg.meshes[0] = collision_object_mesh;
  co_msg.mesh_poses[0].position.x = pose.position.x;   //[m]
  co_msg.mesh_poses[0].position.y = pose.position.y;   //[m]
  co_msg.mesh_poses[0].position.z = pose.position.z;
  co_msg.mesh_poses[0].orientation.w= pose.orientation.w;
  co_msg.mesh_poses[0].orientation.x= pose.orientation.x;
  co_msg.mesh_poses[0].orientation.y= pose.orientation.y;
  co_msg.mesh_poses[0].orientation.z= pose.orientation.z;

  co_msg.operation = co_msg.ADD;

//   return *co_msg;
}


}

#endif
