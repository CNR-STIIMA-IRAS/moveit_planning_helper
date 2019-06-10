#ifndef __ITIA_MVUTILS__H__STRING__
#define __ITIA_MVUTILS__H__STRING__

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <moveit/collision_detection/collision_common.h>

namespace itia
{
namespace mvutils
{
  
/**
 * @fn to_string 
 * @param collision_detection::CollisionResult to string
 * @return string
 */
inline std::string to_string (const collision_detection::CollisionResult& cr )
{
  std::string ret;
  collision_detection::CollisionResult::ContactMap::const_iterator it;
  for(it = cr.contacts.begin(); it != cr.contacts.end(); ++it )
    ret += std::to_string( std::distance( cr.contacts.begin(), it ) ) + "/ " + std::to_string( cr.contacts.size() ) 
        + "# Contact between: " +  it->first.first + " and "  + it->first.second + "\n";
        
  return ret;
}

/**
 * @fn to_string 
 * @param geometry_msgs::Pose to string
 * @return string
 */
inline std::string to_string (const geometry_msgs::Pose& p )
{
  std::string ret = "pos x: " + std::to_string( p.position.x )
                  + " y: "    + std::to_string( p.position.y )
                  + " z: "    + std::to_string( p.position.z )
                  + " orient x: " + std::to_string( p.orientation.x )
                  + " y: "       + std::to_string( p.orientation.y )
                  + " z: "       + std::to_string( p.orientation.z )
                  + " w: "       + std::to_string( p.orientation.w );
  return ret;
}


}
}


#endif