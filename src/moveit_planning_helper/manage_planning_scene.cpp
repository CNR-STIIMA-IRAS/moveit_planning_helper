
#include <moveit_planning_helper/manage_planning_scene.h>
#include <moveit_planning_helper/conversions.h>
#include <numeric>

#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit/move_group/capability_names.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <descartes_moveit/ikfast_moveit_state_adapter.h>
#include <descartes_trajectory/axial_symmetric_pt.h>
#include <descartes_trajectory/cart_trajectory_pt.h>
#include <descartes_trajectory/joint_trajectory_pt.h>
#include <descartes_planner/dense_planner.h>
#include <descartes_utilities/ros_conversions.h>
#include <random>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

std::string DEFAULT      ( ) { return  "\033[0m";             }
std::string RESET        ( ) { return  "\033[0m";             }
std::string BLACK        ( ) { return  "\033[30m";            }
std::string RED          ( ) { return  "\033[31m";            }
std::string GREEN        ( ) { return  "\033[32m";            }
std::string YELLOW       ( ) { return  "\033[33m";            }
std::string BLUE         ( ) { return  "\033[34m";            }
std::string MAGENTA      ( ) { return  "\033[35m";            }
std::string CYAN         ( ) { return  "\033[36m";            }
std::string WHITE        ( ) { return  "\033[37m";            }
std::string BOLDBLACK    ( ) { return  "\033[1m\033[30m";     }
std::string BOLDRED      ( ) { return  "\033[1m\033[31m";     }
std::string BOLDGREEN    ( ) { return  "\033[1m\033[32m";     }
std::string BOLDYELLOW   ( ) { return  "\033[1m\033[33m";     }
std::string BOLDBLUE     ( ) { return  "\033[1m\033[34m";     }
std::string BOLDMAGENTA  ( ) { return  "\033[1m\033[35m";     }
std::string BOLDCYAN     ( ) { return  "\033[1m\033[36m";     }
std::string BOLDWHITE    ( ) { return  "\033[1m\033[37m";     }

namespace moveit_planning_helper
{




std::ostream& operator<<(std::ostream& stream, const Eigen::Affine3d& affine)
{
  Eigen::Quaterniond q( affine.linear() );
  stream.precision(6);
  stream.width (7);
  stream << "\e[1mRotation:\e[0m"    << std::endl << std::fixed << affine.rotation() << std::endl;
  stream << "\e[1mQuaternion:\e[0m"  << std::endl << std::fixed << "x: "<< q.x() << " z: "<< q.y() << " z: "<< q.z() << " w: "<< q.w() << std::endl;
  stream << "\e[1mTranslation:\e[0m" << std::endl << std::fixed << affine.translation().transpose() << std::endl;

  return stream;
}

std::ostream& operator<<(std::ostream& stream, const tf::Transform& transform)
{
  Eigen::Affine3d affine;
  tf::transformTFToEigen( transform, affine );
  stream << affine;
  return stream;
}

std::string to_string(const geometry_msgs::Pose& pose)
{
  std::stringstream str;
  Eigen::Affine3d affine;
  tf::poseMsgToEigen( pose, affine );
  str << affine;
  return str.str();
}

std::string to_string(const Eigen::Affine3d& affine)
{
  std::stringstream str;
  str << affine;
  return str.str();
}

std::string to_string(const tf::Pose& transform)
{
  Eigen::Affine3d affine;
  tf::transformTFToEigen( transform, affine );
  return to_string( affine );
}

void vecToTf ( std::vector<double> *pose ,tf::Pose& transform )
{

    geometry_msgs::Pose p;
    p.position.x = pose->at ( 0 );
    p.position.y = pose->at ( 1 );
    p.position.z = pose->at ( 2 );

    tf::Quaternion q;
    if ( pose->size() == 6 )
    {
        std::cout << "roll: " << pose->at ( 3 ) << ", " << pose->at ( 4 ) <<", " << pose->at ( 5 ) << std::endl;
        q = tf::createQuaternionFromRPY ( pose->at ( 3 ),pose->at ( 4 ),pose->at ( 5 ) );
        tf::quaternionTFToMsg ( q, p.orientation );
        std::cout << q << std::endl;
    }
    else
        q = tf::Quaternion ( pose->at ( 4 ),pose->at ( 5 ),pose->at ( 6 ),pose->at ( 3 ) );

    tf::poseMsgToTF ( p, transform );
}




std::string to_string (const collision_detection::CollisionResult& cr )
{
  std::string ret;
  collision_detection::CollisionResult::ContactMap::const_iterator it;
  for(it = cr.contacts.begin(); it != cr.contacts.end(); ++it )
    ret += std::to_string( std::distance( cr.contacts.begin(), it ) ) + "/ " + std::to_string( cr.contacts.size() ) 
        + "# Contact between: " +  it->first.first + " and "  + it->first.second + "\n";
        
  return ret;
}



std::vector< std::vector<double> > sort( const std::vector< std::vector<double> >& vv, const std::vector<double> & ref )
{
  std::vector< std::vector<double> >  vv_sorted;
  std::vector< double >               dist;
  for( auto v : vv )
  {
    std::vector< double > diff2( v.size( ) );
    std::transform( v.begin(), v.end(), ref.begin(), diff2.begin(), 
                  [&]( double vi, double refi ){ return pow( vi - refi, 2 ); } );
    
    dist.push_back( sqrt( std::accumulate( diff2.begin(), diff2.end(), 0.0 ) ));
  }
  size_t n(0);
  std::vector< size_t > idx( dist.size() );
  std::transform(idx.begin(), idx.end(), idx.begin(), [&](size_t k){ return n++; });
  std::sort(idx.begin(), idx.end(), [&](size_t i1, size_t i2) {return dist.at(i1) < dist.at(i2);} );  
  
  vv_sorted.clear();
  for( size_t i = 0; i < idx.size(); i++ )
  {
    vv_sorted.push_back( vv[ idx[ i ] ] );

  }

  return vv_sorted;
}

std::map<std::string, std::vector< double> > getJointPositions( const moveit::core::RobotState& robot_state, const moveit::core::JointModelGroup*  joint_model_group )
{
  const std::vector<std::string>    joint_names  = joint_model_group->getJointModelNames();

  std::map<std::string, std::vector< double> > robot_state_joints;
  for( auto it = joint_names.begin(); it != joint_names.end(); it++ )
  {
    std::vector<double> positions ( robot_state.getJointPositions( *it )
                                  , robot_state.getJointPositions( *it )  + joint_model_group->getJointModel(*it)->getVariableCount() );
    robot_state_joints[ *it ] = positions; 
  } 

  return robot_state_joints;
}

double normFrobenius( const std::map<std::string, std::vector< double> >& robot_state1, const std::map<std::string, std::vector< double> >& robot_state2 )
{
  double ret = 0.0;
  for( auto it = robot_state1.begin(); it != robot_state1.end() ; it++ )
  {
    if( robot_state2.find(it->first) == robot_state2.end() )
    {
      ROS_ERROR("Different Joint Model Groups!!!");
      return -1;
    }
    
    std::vector< double > joint1 = it->second;
    std::vector< double > joint2 = robot_state2.at( it->first );
    if( joint1.size() != joint2.size() )
    {
      ROS_ERROR("Error in joint dimensions!!!");
      return -1;
    }
    
    for( size_t i=0; i< joint1.size(); i++ )
      ret += std::pow( joint1[i] -  joint2[i], 2 );
    
    ret = std::sqrt( ret );
  }
  
  return ret;
}

void rvizDisplayPath( ros::NodeHandle&                                        nh
                    , const moveit::planning_interface::MoveGroupInterface::Plan&   plan
                    , const std::string&                                      ns )
{
  ros::Publisher display_publisher = nh.advertise<moveit_msgs::DisplayTrajectory>(ns+"display_planned_path", 1, true);
  moveit_msgs::DisplayTrajectory display_trajectory;
              
  display_trajectory.trajectory_start = plan.start_state_;
  display_trajectory.trajectory.push_back(plan.trajectory_);
  display_publisher.publish(display_trajectory);
  sleep(5.0);
 
}

bool getParam(const ros::NodeHandle& nh, std::string name_param, geometry_msgs::Pose& pose, std::string param )
{ 
  bool ret = true;
  if( param=="pose-geometry-msg")
  {
    ret &= nh.getParam(name_param+"/position/x", pose.position.x);
    ret &= nh.getParam(name_param+"/position/y", pose.position.y);;
    ret &= nh.getParam(name_param+"/position/z", pose.position.z);
    ret &= nh.getParam(name_param+"/orientation/w", pose.orientation.w);
    ret &= nh.getParam(name_param+"/orientation/x", pose.orientation.x);
    ret &= nh.getParam(name_param+"/orientation/y", pose.orientation.y);
    ret &= nh.getParam(name_param+"/orientation/z", pose.orientation.z);
  }
  else if(param=="pose-vector")
  {
    std::vector<double> v;
    ret &= nh.getParam(name_param, v);
    pose.position.x = v[0];
    pose.position.y = v[1];
    pose.position.z = v[2];
    pose.orientation.x = v[3];
    pose.orientation.y = v[4];
    pose.orientation.z = v[5];
    pose.orientation.w = v[6];
  }
  return ret;
}
  
robot_state::RobotState getRobotState ( ros::NodeHandle&                       nh
                                      , const moveit::core::RobotModelConstPtr robot_model
                                      , const std::string&                     ns
                                      ) 
{
  
  ros::ServiceClient planning_scene_service;
  planning_scene_service = nh.serviceClient<moveit_msgs::GetPlanningScene>(ns + "/" + move_group::GET_PLANNING_SCENE_SERVICE_NAME);
  if( !planning_scene_service.waitForExistence( ros::Duration(5.0) ) )
  {
    ROS_ERROR("getRobotState Failed: service '%s' does not exist ", (ns + "/" + move_group::GET_PLANNING_SCENE_SERVICE_NAME).c_str() );
  }
  
  robot_state::RobotState* ret = new robot_state::RobotState( robot_model );
  
  moveit_msgs::GetPlanningScene::Request request;
  moveit_msgs::GetPlanningScene::Response response;
    
  request.components.components = request.components.ROBOT_STATE;
  if (!planning_scene_service.call(request, response))
  {
    ROS_WARN("Could not call planning scene service to get object names");
  }

  moveit::core::robotStateMsgToRobotState(response.scene.robot_state, *ret);
  
  return *ret;
}


void getPlanningScene ( ros::NodeHandle& nh
                      , const std::string& ns
                      , moveit_msgs::PlanningScene& planning_scene_msgs)
{
  ros::ServiceClient planning_scene_service;
  planning_scene_service = nh.serviceClient<moveit_msgs::GetPlanningScene>(ns + "/" +move_group::GET_PLANNING_SCENE_SERVICE_NAME);
  if(!planning_scene_service.waitForExistence( ros::Duration(5.0) ) )
  {
    ROS_ERROR("getPlanningScene Failed: service '%s' does not exist ", (ns + "/" +move_group::GET_PLANNING_SCENE_SERVICE_NAME).c_str() );
  }

  
  { /// ROBOT_STATE
    moveit_msgs::GetPlanningScene::Request request;
    moveit_msgs::GetPlanningScene::Response response;

    request.components.components = request.components.ROBOT_STATE;
    if (!planning_scene_service.call(request, response))
    {
      ROS_WARN("Could not call planning scene service to get object names");
    }


    planning_scene_msgs.name = response.scene.name;
    planning_scene_msgs.robot_state = response.scene.robot_state;
  }
  { // WORLD_OBJECT_GEOMETRY && WORLD_OBJECT_NAMES
    moveit_msgs::GetPlanningScene::Request request;
    moveit_msgs::GetPlanningScene::Response response;

    request.components.components = request.components.WORLD_OBJECT_GEOMETRY;
    if (!planning_scene_service.call(request, response))
    {
      ROS_WARN("Could not call planning scene service to get object names");
    }
    planning_scene_msgs.world.collision_objects = response.scene.world.collision_objects;
  }
  { // OCTOMAP
    moveit_msgs::GetPlanningScene::Request request;
    moveit_msgs::GetPlanningScene::Response response;

    request.components.components = request.components.OCTOMAP;
    if (!planning_scene_service.call(request, response))
    {
      ROS_WARN("Could not call planning scene service to get object names");
    }
    planning_scene_msgs.world.octomap = response.scene.world.octomap;
  }
  { // TRANSFORMS
    moveit_msgs::GetPlanningScene::Request request;
    moveit_msgs::GetPlanningScene::Response response;

    request.components.components = request.components.TRANSFORMS;
    if (!planning_scene_service.call(request, response))
    {
      ROS_WARN("Could not call planning scene service to get object names");
    }
    planning_scene_msgs.fixed_frame_transforms = response.scene.fixed_frame_transforms;
  }
  { // ALLOWED_COLLISION_MATRIX
    moveit_msgs::GetPlanningScene::Request request;
    moveit_msgs::GetPlanningScene::Response response;

    request.components.components = request.components.ALLOWED_COLLISION_MATRIX;
    if (!planning_scene_service.call(request, response))
    {
      ROS_WARN("Could not call planning scene service to get object names");
    }
    planning_scene_msgs.allowed_collision_matrix = response.scene.allowed_collision_matrix;
  }
  { // LINK_PADDING_AND_SCALING
    moveit_msgs::GetPlanningScene::Request request;
    moveit_msgs::GetPlanningScene::Response response;

    request.components.components = request.components.LINK_PADDING_AND_SCALING;
    if (!planning_scene_service.call(request, response))
    {
      ROS_WARN("Could not call planning scene service to get object names");
    }
    planning_scene_msgs.link_padding = response.scene.link_padding;
    planning_scene_msgs.link_scale   = response.scene.link_scale;
  }
  { // OBJECT_COLORS
    moveit_msgs::GetPlanningScene::Request request;
    moveit_msgs::GetPlanningScene::Response response;

    request.components.components = request.components.LINK_PADDING_AND_SCALING;
    if (!planning_scene_service.call(request, response))
    {
      ROS_WARN("Could not call planning scene service to get object names");
    }
    planning_scene_msgs.object_colors = response.scene.object_colors;
  }

  return;
}


void getPlanningScene   ( ros::NodeHandle& nh
                        , planning_scene::PlanningScenePtr& ret )
{
    ros::ServiceClient planning_scene_service;
    planning_scene_service = nh.serviceClient<moveit_msgs::GetPlanningScene> ( "get_planning_scene" );
    if ( !planning_scene_service.waitForExistence ( ros::Duration ( 5.0 ) ) )
    {
        ROS_ERROR ( "getPlanningScene Failed: service 'get_planning_scene ' does not exist" );
    }

    moveit_msgs::PlanningScene planning_scene_msgs;

    {
        /// ROBOT_STATE
        moveit_msgs::GetPlanningScene::Request request;
        moveit_msgs::GetPlanningScene::Response response;

        request.components.components = request.components.ROBOT_STATE;
        if ( !planning_scene_service.call ( request, response ) )
        {
            ROS_WARN ( "Could not call planning scene service to get object names" );
        }


        planning_scene_msgs.name = response.scene.name;
        planning_scene_msgs.robot_state = response.scene.robot_state;
    }
    {
        // WORLD_OBJECT_GEOMETRY && WORLD_OBJECT_NAMES
        moveit_msgs::GetPlanningScene::Request request;
        moveit_msgs::GetPlanningScene::Response response;

        request.components.components = request.components.WORLD_OBJECT_GEOMETRY;
        if ( !planning_scene_service.call ( request, response ) )
        {
            ROS_WARN ( "Could not call planning scene service to get object names" );
        }
        planning_scene_msgs.world.collision_objects = response.scene.world.collision_objects;
    }
    {
        // OCTOMAP
        moveit_msgs::GetPlanningScene::Request request;
        moveit_msgs::GetPlanningScene::Response response;

        request.components.components = request.components.OCTOMAP;
        if ( !planning_scene_service.call ( request, response ) )
        {
            ROS_WARN ( "Could not call planning scene service to get object names" );
        }
        planning_scene_msgs.world.octomap = response.scene.world.octomap;
    }
    {
        // TRANSFORMS
        moveit_msgs::GetPlanningScene::Request request;
        moveit_msgs::GetPlanningScene::Response response;

        request.components.components = request.components.TRANSFORMS;
        if ( !planning_scene_service.call ( request, response ) )
        {
            ROS_WARN ( "Could not call planning scene service to get object names" );
        }
        planning_scene_msgs.fixed_frame_transforms = response.scene.fixed_frame_transforms;
    }
    {
        // ALLOWED_COLLISION_MATRIX
        moveit_msgs::GetPlanningScene::Request request;
        moveit_msgs::GetPlanningScene::Response response;

        request.components.components = request.components.ALLOWED_COLLISION_MATRIX;
        if ( !planning_scene_service.call ( request, response ) )
        {
            ROS_WARN ( "Could not call planning scene service to get object names" );
        }
        planning_scene_msgs.allowed_collision_matrix = response.scene.allowed_collision_matrix;
    }
    {
        // LINK_PADDING_AND_SCALING
        moveit_msgs::GetPlanningScene::Request request;
        moveit_msgs::GetPlanningScene::Response response;

        request.components.components = request.components.LINK_PADDING_AND_SCALING;
        if ( !planning_scene_service.call ( request, response ) )
        {
            ROS_WARN ( "Could not call planning scene service to get object names" );
        }
        planning_scene_msgs.link_padding = response.scene.link_padding;
        planning_scene_msgs.link_scale   = response.scene.link_scale;
    }
    {
        // OBJECT_COLORS
        moveit_msgs::GetPlanningScene::Request request;
        moveit_msgs::GetPlanningScene::Response response;

        request.components.components = request.components.LINK_PADDING_AND_SCALING;
        if ( !planning_scene_service.call ( request, response ) )
        {
            ROS_WARN ( "Could not call planning scene service to get object names" );
        }
        planning_scene_msgs.object_colors = response.scene.object_colors;
    }

    ret->setPlanningSceneMsg ( planning_scene_msgs );

    return;
}


bool setRobotStateNH( ros::NodeHandle&                          nh
                    , const moveit::core::RobotState&           robot_state
                    , const moveit::core::RobotModelConstPtr    robot_model
                    , const std::string&                        group_name
                    , const std::string&                        ns )
{
    
    ros::Publisher planning_scene_diff_publisher = nh.advertise<moveit_msgs::PlanningScene>( ns + "/planning_scene", 1);
  
    planning_scene::PlanningScenePtr      planning_scene( new planning_scene::PlanningScene( robot_model ) );
    getPlanningScene(nh, planning_scene );
    
    const moveit::core::JointModelGroup*  joint_model_group = robot_model->getJointModelGroup(group_name);
    const std::vector<std::string>        joint_names       = joint_model_group->getJointModelNames();
    
    planning_scene->setCurrentState(robot_state);
    
    std::map<std::string, std::vector< double> > robot_state_joints = getJointPositions( robot_state, joint_model_group );

    ////////////////////////////////////////////////////////////////////////
    // ADD
    moveit_msgs::PlanningScene planning_scene_msg;
    
    planning_scene->getPlanningSceneMsg( planning_scene_msg );
    
    planning_scene_msg.is_diff = true;
    
    //////////////////////////////////////////////////////////////////
    // PUBLISH AND CHECK
    double timeout = 5.0;
    int num_subscriber_connected = 0;
    ros::Time t = ros::Time::now();
    do {
      num_subscriber_connected = planning_scene_diff_publisher.getNumSubscribers();
      if( num_subscriber_connected )
        break;

      if( (timeout/2.0 > 0) && ( (ros::Time::now() - t ).toSec() > timeout) )
      {
        ROS_WARN("Failed, none subscriber is connected to the planning_scene publisher");
        return false;
      }
      ros::Duration(0.01).sleep();
    } while( ros::ok( ) );
    
    planning_scene_diff_publisher.publish(planning_scene_msg);
    
    
    t = ros::Time::now();
    do {
      moveit::core::RobotState current_robot_state = getRobotState( nh, robot_model, ns );
      
      std::map<std::string, std::vector< double> > robot_current_state_joints = getJointPositions( robot_state, joint_model_group );
      
      
      double ret = normFrobenius( robot_state_joints, robot_current_state_joints );

      if( ret < 1e-3 )
        break;
      
      if( (timeout > 0) && ( (ros::Time::now() - t ).toSec() > timeout/2.0) )
      {
        ROS_WARN("Failed, none subscriber is connected to the planning_scene publisher");
        return false;
      }
      ros::Duration(0.01).sleep();
    } while( ros::ok( ) );
    
    return true;
}



std::shared_ptr<moveit_msgs::CollisionObject> toCollisionObject( const std::string      &collisionObjID
                                                               , const std::string      &path_to_mesh
                                                               , const std::string      &reference_frame
                                                               , const tf::Pose         &pose
                                                               , const Eigen::Vector3d   scale )
{

    std::shared_ptr<moveit_msgs::CollisionObject> collision_object(new moveit_msgs::CollisionObject );
    collision_object->id = collisionObjID;
    shapes::Mesh* m = shapes::createMeshFromResource ( path_to_mesh, scale );

    shape_msgs::Mesh mesh;
    shapes::ShapeMsg mesh_msg;
    shapes::constructMsgFromShape ( m, mesh_msg );
    mesh = boost::get<shape_msgs::Mesh> ( mesh_msg );

    collision_object->meshes.resize ( 1 );
    collision_object->mesh_poses.resize ( 1 );
    collision_object->meshes[0] = mesh;
    collision_object->header.frame_id = reference_frame;

    geometry_msgs::Pose pose_msg;
    tf::poseTFToMsg ( pose, pose_msg );

    collision_object->mesh_poses[0] = pose_msg;

    collision_object->meshes.push_back ( mesh );
    collision_object->mesh_poses.push_back ( collision_object->mesh_poses[0] );
    collision_object->operation = collision_object->ADD;

    return collision_object;

}

std::shared_ptr< moveit_msgs::CollisionObject > toCollisionObject ( const std::string&         collisionObjID
                                                                  , const std::string&         path_to_mesh
                                                                  , const std::string&         reference_frame
                                                                  , const geometry_msgs::Pose& pose 
                                                                  , const Eigen::Vector3d      scale)
{
  tf::Pose p;
  tf::poseMsgToTF( pose, p );
  return toCollisionObject(collisionObjID, path_to_mesh, reference_frame, p, scale );
}


std::shared_ptr< moveit_msgs::CollisionObject > toCollisionObject ( const std::string&         collisionObjID
                                                                  , const std::string&         path_to_mesh
                                                                  , const std::string&         reference_frame
                                                                  , const Eigen::Affine3d&     pose 
                                                                  , const Eigen::Vector3d      scale)
{
  tf::Pose p;
  tf::transformEigenToTF( pose, p );
  return toCollisionObject(collisionObjID, path_to_mesh, reference_frame, p, scale );
}


bool applyAndCheckPS( ros::NodeHandle nh
                    , std::vector<moveit_msgs::CollisionObject> cov
                    , std::vector<moveit_msgs::ObjectColor> colors , ros::Duration timeout)
{

    ros::Publisher planning_scene_diff_publisher = nh.advertise<moveit_msgs::PlanningScene> ( "planning_scene", 1 );

    while ( planning_scene_diff_publisher.getNumSubscribers() < 1 )
    {
        ros::WallDuration sleep_t ( 0.5 );
        sleep_t.sleep();
    }

    moveit_msgs::PlanningScene planning_scene_msg;

    for(auto const & color : colors )
      planning_scene_msg.object_colors.push_back ( color );


    for ( moveit_msgs::CollisionObject co: cov )
        planning_scene_msg.world.collision_objects.push_back ( co );

    planning_scene_msg.is_diff = true;

    ROS_INFO_STREAM("Update planning scene");
    ros::ServiceClient planning_scene_diff_client = nh.serviceClient<moveit_msgs::ApplyPlanningScene> ( "apply_planning_scene" );
    planning_scene_diff_client.waitForExistence();

    moveit_msgs::ApplyPlanningScene srv;
    srv.request.scene = planning_scene_msg;
    planning_scene_diff_client.call ( srv );

    ROS_INFO_STREAM("Wait for updated planning scene...");
    moveit::planning_interface::PlanningSceneInterface  planning_scene_interface;
    ros::Time st = ros::Time::now();
    while( ros::ok() )
    {
      auto const obj_names = planning_scene_interface.getKnownObjectNames( );
      bool loaded = true;
      for( const auto & co : cov )
      {
        auto const it = std::find_if( obj_names.begin(), obj_names.end(), [ & ]( const std::string s ) { return co.id == s;} );
        loaded &= (obj_names.end() != it);

      }
      if( loaded )
      {
        ROS_INFO_STREAM("Updated planning scene.");
        break;
      }
      else
      {
        if( (ros::Time::now() - st) > timeout)
        {
          ROS_FATAL_STREAM("Timeout expired.");
          return false;
        }
        ROS_INFO_STREAM_THROTTLE(2,"Wait for updated planning scene...");
      }
      ros::Duration(0.2).sleep();
    }
    return true;
}




RobotStateExtended::RobotStateExtended( const ros::NodeHandle&                  nh
                                      , const moveit::core::RobotModelConstPtr  robot_model
                                      , const std::string&                      kin_group
                                      , const std::string&                      last_link
                                      , const std::string&                      ns ) 
  : nh_         (nh)
  , ns_         (ns)
  , robot_model_(robot_model)
  , kin_group_  (kin_group)
  , last_link_  (last_link)
  , joint_model_group_ ( robot_model_->getJointModelGroup(kin_group_) )
{                   
  robot_state_joints_names_ = robot_model->getJointModelGroup(kin_group)->getActiveJointModelNames();
  
}

const std::vector< robot_state::RobotState >& RobotStateExtended::getRobotStateConfigurations( ) const
{ 
  return robot_state_; 
}

const std::vector< std::vector<double> >& RobotStateExtended::getRobotStateJointConfigurations( ) const
{ 
  return robot_state_joints_; 
}

const std::vector< std::string >& RobotStateExtended::getRobotStateJointsNames( ) const
{ 
  return robot_state_joints_names_; 
}

bool RobotStateExtended::ikine( const kinematics::KinematicsBaseConstPtr& ikine_solver 
                              , const geometry_msgs::Pose&                pose
                              , std::vector <std::vector <double> >*      solutions
                              , const std::vector<double>&                first_tentative_ik_seed_state
                              , const bool                                verbose ) const
{  
  assert( solutions != NULL );
  solutions->clear();
  
  // const kinematics::KinematicsBaseConstPtr&   solver = robot_model_->getJointModelGroup(kin_group_)->getSolverInstance();  

  if(verbose) ROS_INFO_STREAM( BOLDYELLOW( ) << "Configuration: \n" << RESET() << pose );   

  std::vector< std::vector<double> >  q;
  kinematics::KinematicsResult        result;
  kinematics::KinematicsQueryOptions  options;
  
  std::vector<double> ik_seed_state = first_tentative_ik_seed_state;
  
  assert( ikine_solver );
  std::cout << ikine_solver.get() << std::endl;
  if(verbose) ROS_INFO_STREAM( BOLDYELLOW( ) << "Get Position: \n" << RESET() << pose );   
  ikine_solver->getPositionIK( std::vector<geometry_msgs::Pose>{ pose }, ik_seed_state, q, result, options ); 
  
  int i=0;
  if( q.size() > 0 )
  {
    *solutions = q;
    if(verbose) 
    {
      for( auto const & s : q )
      {
        std::string ret = "[ "; for( auto const d : s ) ret += std::to_string(d) + " "; ret +="]";
        ROS_INFO_STREAM( BOLDYELLOW( ) << i++ <<"/" << q.size() << "# Solution: \n" << RESET() << ret );     
      }
    }
    
    return true;
  }
  
  if(verbose) ROS_WARN_STREAM( BOLDYELLOW( ) << "no ik solutions" );   

  return false;
}

bool RobotStateExtended::fkine( const kinematics::KinematicsBaseConstPtr&    ikine_solver 
                              , const std::vector<double>& joints_positions
                              , geometry_msgs::Pose* pose
                              , std::vector <std::vector <double> >*  equivalent_solutions
                              , const bool verbose ) const 
{
  assert( joints_positions.size() == robot_state_joints_names_.size() );
  assert( pose != NULL );
  if(verbose) ROS_INFO("set robot_state");
  moveit::core::RobotState robot_state( robot_model_ );
  if(verbose) ROS_INFO("Joint Pos: %s", to_string( joints_positions ).c_str() );
  robot_state.setJointGroupPositions (robot_model_->getJointModelGroup(kin_group_), joints_positions );
  robot_state.update();
  
  if(verbose) ROS_INFO("Last Link: %s", last_link_.c_str() );
  *pose = moveit_planning_helper::robotStateToPoseMsg(robot_state, last_link_); 
  
  if(verbose) ROS_INFO_STREAM("Last Link: " << *pose);
  
  if( equivalent_solutions != NULL )
    return ikine( ikine_solver, *pose, equivalent_solutions, joints_positions, verbose );
  
  return true;
}

bool RobotStateExtended::fkine( const kinematics::KinematicsBaseConstPtr& ikine_solver 
                              , const std::map<std::string,double>& joints_positions
                              , geometry_msgs::Pose* pose
                              , std::vector <std::vector <double> >*  equivalent_solutions
                              , const bool verbose ) const
{
  assert( joints_positions.size() == robot_state_joints_names_.size() );
  std::vector<double> q;
  for( size_t j=0; j<robot_state_joints_names_.size();j++)
  {
    auto it = joints_positions.find(robot_state_joints_names_[j]);
    if( it != joints_positions.end() )
      q.push_back( it->second  );
  }
  assert( q.size() == robot_state_joints_names_.size() );
  return fkine( ikine_solver, q, pose, equivalent_solutions, verbose );
}

bool RobotStateExtended::checkCollision ( const std::vector<double>&        joints_position
                                        , planning_scene::PlanningScenePtr  planning_scene
                                        , bool                              verbose )
{
  collision_detection::CollisionResult collision_result;    
  collision_result.clear();
  
  moveit::core::RobotState check_state( robot_model_ ); 
  check_state.setJointGroupPositions(robot_model_->getJointModelGroup(kin_group_), joints_position);
  check_state.update();
  
  collision_detection::CollisionRequest collision_request;    
  collision_request.contacts = true;
  collision_request.max_contacts = 1000;
  planning_scene->checkCollision(collision_request, collision_result, check_state, planning_scene->getAllowedCollisionMatrix());
  
  if(collision_result.collision /*&& verbose*/ )
  {
    ROS_INFO(" %sIn collision ", BOLDRED().c_str() );
    collision_detection::CollisionResult::ContactMap::const_iterator it;
    for(it = collision_result.contacts.begin(); it != collision_result.contacts.end(); ++it)
      ROS_INFO(" - Contact between: %s and %s", it->first.first.c_str(), it->first.second.c_str());
  }
  else if ( verbose )
    ROS_INFO("%s Not in collision ", BOLDGREEN().c_str() );

  return collision_result.collision == 0;
}

bool RobotStateExtended::setRobotState  ( const std::vector<double>&        joints_positions
                                        , planning_scene::PlanningScenePtr  planning_scene
                                        , bool                              calc_equivalent_joint_configurations
                                        , bool                              verbose)
{
  geometry_msgs::Pose pose;
  std::vector <std::vector <double> >  solutions_to_check_sorted;
  if( calc_equivalent_joint_configurations )
  {
    std::vector <std::vector <double> >  equivalent_solutions;
    if( !fkine( robot_model_->getJointModelGroup(kin_group_)->getSolverInstance(),joints_positions, &pose, &equivalent_solutions, verbose) )
    {
      ROS_ERROR_STREAM( BOLDRED() << "Fkine failed:" );
      fkine( robot_model_->getJointModelGroup(kin_group_)->getSolverInstance(),joints_positions, &pose, &equivalent_solutions, true) ;
      return false;
    }
    
    if( verbose ) ROS_INFO_STREAM( "Checking all the equivalent joint solutions" );
    solutions_to_check_sorted = sort( equivalent_solutions, joints_positions);
  }
  else
  {
    if( !fkine(robot_model_->getJointModelGroup(kin_group_)->getSolverInstance(), joints_positions, &pose, NULL, verbose) )
    {
      ROS_ERROR_STREAM( BOLDRED( ) << "Fkine failed:" );
      fkine( robot_model_->getJointModelGroup(kin_group_)->getSolverInstance(), joints_positions, &pose, NULL, true) ;
      return false;
    }
    solutions_to_check_sorted.push_back(joints_positions);
  }
    
  
  robot_state_joints_.clear();
  robot_state_.clear();
  for( size_t i=0; i< solutions_to_check_sorted.size(); i++ )
  {
    if( verbose ) 
    {
      ROS_INFO_STREAM( "Checking " << i+1 <<"/"<< solutions_to_check_sorted.size() << " conf " << to_string( solutions_to_check_sorted[i] )  );
    }
    
    if( checkCollision( solutions_to_check_sorted[i], planning_scene, verbose ) )
    {
      robot_state_joints_.push_back( solutions_to_check_sorted[i] ); 
      robot_state::RobotState rbs( robot_model_ );
      rbs.setJointGroupPositions(joint_model_group_, solutions_to_check_sorted[i] );
      rbs.update();
      robot_state_.push_back( rbs );
    }
  }
  
  if( robot_state_joints_.size() == 0 )
  {
    ROS_ERROR_STREAM( BOLDRED( ) << "Collision Check failed." );
    return false;
  }
  
  if(verbose)
  {
    geometry_msgs::Pose ee_msgs = moveit_planning_helper::robotStateToPoseMsg(robot_state_.front(), last_link_);  
    ROS_INFO_STREAM( BOLDYELLOW( ) << "Pose: " << RESET() << to_string( ee_msgs) ); 
  }
  return true;
}


bool RobotStateExtended::setRobotState( const robot_state::RobotState&    robot_state
                                      , planning_scene::PlanningScenePtr  planning_scene
                                      , bool                              calc_equivalent_joint_configurations  
                                      , bool                              verbose                               )
{
  std::vector< double > robot_state_joints;
  robot_state.copyJointGroupPositions(joint_model_group_, robot_state_joints);

  return setRobotState( robot_state_joints, planning_scene, calc_equivalent_joint_configurations, verbose );
}


bool RobotStateExtended::checkRobotState( planning_scene::PlanningScenePtr  planning_scene
                                        , bool                              calc_equivalent_joint_configurations  
                                        , bool                              verbose                               )
{
  if( robot_state_.size() == 0 )
    return false; 
  robot_state::RobotState robot_state = robot_state_.front();
  return setRobotState( robot_state, planning_scene, calc_equivalent_joint_configurations, verbose );
}
  
bool RobotStateExtended::setRobotState  ( const std::vector<double>&   joints_positions
                                        , bool                         calc_equivalent_joint_configurations
                                        , bool                         verbose)
{
  moveit::core::RobotState check_state( robot_model_ ); 
  
  check_state.setJointGroupPositions(robot_model_->getJointModelGroup(kin_group_), joints_positions);
  
  setRobotStateNH(nh_, check_state, robot_model_, kin_group_, ns_ );  
  setRobotStateNH(nh_, check_state, robot_model_, kin_group_, ns_ );  
  setRobotStateNH(nh_, check_state, robot_model_, kin_group_, ns_ );  
  planning_scene::PlanningScenePtr planning_scene( new planning_scene::PlanningScene( robot_model_ ) );
  moveit_msgs::PlanningScene planning_scene_msgs;
  getPlanningScene( nh_, ns_, planning_scene_msgs);
  planning_scene->setPlanningSceneMsg(planning_scene_msgs);
  
  bool ok = setRobotState( joints_positions, planning_scene, calc_equivalent_joint_configurations, verbose);
  
  return ok; 
}


bool RobotStateExtended::setRobotState  ( const robot_state::RobotState& robot_state
                                        , bool                           calc_equivalent_joint_configurations 
                                        , bool                           verbose )
{
  setRobotStateNH(nh_, robot_state, robot_model_, kin_group_ , ns_ );  
  
  planning_scene::PlanningScenePtr planning_scene( new planning_scene::PlanningScene( robot_model_ ) );
  moveit_msgs::PlanningScene planning_scene_msgs;
  getPlanningScene( nh_, ns_, planning_scene_msgs);
  planning_scene->setPlanningSceneMsg(planning_scene_msgs);
  
  bool ok = setRobotState( robot_state, planning_scene, calc_equivalent_joint_configurations, verbose );
  planning_scene.reset();
  return ok;
}

bool RobotStateExtended::checkRobotState( bool  calc_equivalent_joint_configurations 
                                        , bool  verbose )
{
  if( robot_state_.size() == 0 )
    return false; 
  robot_state::RobotState robot_state = robot_state_.front();
  return setRobotState( robot_state, calc_equivalent_joint_configurations, verbose );
}

bool RobotStateExtended::updateTF ( int pub_time, const int idx_configuration )
{
  ros::Publisher joint_pub = nh_.advertise<sensor_msgs::JointState>("/remap_joint_states", 1);   //for the update of joitns poistion
  sensor_msgs::JointState joint_state;  
  
  joint_state.name.resize( robot_state_joints_names_.size() ) ;
  joint_state.position.resize( robot_state_joints_names_.size()  );
  
  for (size_t iJoint=0; iJoint<joint_state.name.size(); iJoint++)
  {
    joint_state.name[iJoint] = robot_state_joints_names_.at(iJoint);
    joint_state.position[iJoint] = *(robot_state_[idx_configuration].getJointPositions(joint_state.name[iJoint]));
  }
  for (int i =0; i< pub_time; i++)
  {
    joint_state.header.stamp = ros::Time::now();
    joint_pub.publish(joint_state);
    ros::spinOnce();
    ros::Duration(0.1).sleep(); 
  } 
  return true;
}


bool plan ( const geometry_msgs::Pose&                            target_pose
          , const std::string&                                    planning_goup
          , robot_model::RobotModelPtr&                           robot_model
          , moveit::planning_interface::MoveGroupInterface&       move_group
          , std::vector<double>&                                  target_jpos
          , moveit::planning_interface::MoveGroupInterface::Plan& my_plan){

  kinematics::KinematicsBasePtr solver = robot_model->getJointModelGroup ( planning_goup )->getSolverInstance();
  moveit_msgs::MoveItErrorCodes error_code;
  if ( !solver->getPositionIK ( target_pose, std::vector<double>(7,0), target_jpos, error_code ) )
  {
    ROS_ERROR("solver failed");
    return false;
  }

  move_group.setJointValueTarget ( target_jpos );
  moveit::planning_interface::MoveItErrorCode err = move_group.plan ( my_plan );
  ROS_INFO("Plan return code: %s",  getMoveitErrorCodesIds( ).at( int( err.val ) ).c_str() );
  if ( !err == moveit::planning_interface::MoveItErrorCode::SUCCESS )
  {
    ROS_ERROR("planning error !! %s", getMoveitErrorCodesIds( ).at( int( err.val ) ).c_str() );
    return false;
  }
  return true;
}


bool plan ( const std::vector<double>                             &target_jpos
          , moveit::planning_interface::MoveGroupInterface        &move_group
          , moveit::planning_interface::MoveGroupInterface::Plan  &my_plan )
{
  move_group.setJointValueTarget ( target_jpos );

  moveit::planning_interface::MoveItErrorCode err = move_group.plan ( my_plan );
  ROS_INFO("Plan return code: %s",  getMoveitErrorCodesIds().at( int( err.val ) ).c_str() );
  if ( !err == moveit::planning_interface::MoveItErrorCode::SUCCESS )
  {
    ROS_ERROR("planning error !! %s", getMoveitErrorCodesIds().at( int( err.val ) ).c_str() );
    return false;
  }
  return true;
}


bool cartesian(const geometry_msgs::Pose                              &starting_pose
              , const geometry_msgs::Pose                             &target_pose
              , const tf::StampedTransform                            &worldToRobot
              , const double                                          &default_velocity
              , moveit::planning_interface::MoveGroupInterface        &move_group
              , moveit::planning_interface::MoveGroupInterface::Plan  &my_plan )
{
  std::vector<geometry_msgs::Pose> waypoints;

  tf::Pose sp,tp;

  tf::poseMsgToTF(starting_pose,sp);
  tf::poseMsgToTF(target_pose,tp);
  sp = worldToRobot * sp;
  tp = worldToRobot * tp;

  geometry_msgs::Pose s_p,t_p;

  tf::poseTFToMsg(sp,s_p);
  tf::poseTFToMsg(tp,t_p);

  waypoints.push_back ( s_p);
  waypoints.push_back ( t_p);

  move_group.setMaxVelocityScalingFactor ( default_velocity );

  moveit_msgs::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  double fraction = move_group.computeCartesianPath ( waypoints, eef_step, jump_threshold, trajectory );

  std::cout<<"fraction : "<<fraction<<std::endl;
  if( fraction < 0.95 || trajectory.joint_trajectory.points.size() == 0 )
  {
    ROS_ERROR("Descartes failes in planning a cartesian plan");
    return false;
  }
  trajectory.joint_trajectory.points.erase ( trajectory.joint_trajectory.points.begin() );

  for(int i=0; i<trajectory.joint_trajectory.points.size(); i++)
    std::cout<< trajectory.joint_trajectory.points[i].time_from_start<<std::endl;

  my_plan.planning_time_ = 0;
  my_plan.trajectory_ = trajectory;
  return true;
}


bool descartes( const geometry_msgs::Pose                                     &starting_pose
              , const geometry_msgs::Pose                                     &target_pose
              , const std::vector<std::string>                                &joint_names
              , boost::shared_ptr<descartes_moveit::IkFastMoveitStateAdapter> &robot_model
              , descartes_planner::DensePlanner                               &planner
              , moveit::planning_interface::MoveGroupInterface::Plan          &my_plan
              , std::vector<double>                                           starting_point){

  unsigned int npnts = 25;

  std::vector<descartes_core::TrajectoryPtPtr> way_points;
  if( starting_point.size() > 0 )
  {
    descartes_core::TrajectoryPtPtr p( new descartes_trajectory::JointTrajectoryPt( starting_point ) );
    way_points.push_back( p );
  }

  Eigen::Affine3d starting_pos, end_pos;
  tf::poseMsgToEigen ( starting_pose, starting_pos );
  tf::poseMsgToEigen ( target_pose,   end_pos );

  std::vector<double> lower_deviation ( 6,0 );
  std::vector<double> upper_deviation ( 6,0 );

  auto w  = descartes::createLinearTrajectory ( starting_pos
                                              , end_pos
                                              , lower_deviation
                                              , upper_deviation,npnts
                                              , 0.005
                                              , M_PI*0.05
                                              , 10 );
  way_points.insert( way_points.end(), w.begin(), w.end() );

  std::vector<std::vector<double> > seeds;

  ROS_INFO("Computing seeds");
  const int n_random_seed = 200;
  std::random_device rnd_device;
  std::mt19937 mersenne_engine {rnd_device()};  // Generates random integers
  std::uniform_real_distribution<> dist {-M_PI/2.0, M_PI/2.0};
  for( int i=0; i<n_random_seed; i++)
  {
    std::vector<double> seed ( 7,0.0 );
    std::generate(std::begin(seed), std::end(seed), [&dist, &mersenne_engine](){ return dist(mersenne_engine); });
    seeds.push_back(seed);
  }


  robot_model->setSeedStates(seeds);
  if ( !planner.planPath ( way_points ) )
  {
    ROS_ERROR ( "Could not solve for a valid path" );
    return false;
  }

  std::vector<descartes_core::TrajectoryPtPtr> result;
  if ( !planner.getPath ( result ) )
  {
    ROS_ERROR ( "Descarte fails in get path" );
    return false;
  }

  trajectory_msgs::JointTrajectory cart_trj;
  cart_trj.joint_names = joint_names;
  bool okk = descartes_utilities::toRosJointPoints ( *robot_model,result,0.5,cart_trj.points );
  if ( !okk )
  {
    ROS_ERROR ( "Descarte fails convert trajectory" );
    return false;
  }

  my_plan.trajectory_.joint_trajectory  = cart_trj;
  return true;
}


bool setRobotState( ros::NodeHandle& nh, const std::vector<double>& joint_values, moveit::planning_interface::MoveGroupInterface& move_group )
{

  ros::Publisher move_robot_to_start_state = nh.advertise<sensor_msgs::JointState>("move_group/fake_controller_joint_states", 1000);
  sensor_msgs::JointState start_state_msg;

  robot_model::RobotModelConstPtr robot_model( (new robot_model_loader::RobotModelLoader( "robot_description" ))->getModel() );
  robot_state::RobotState robot_state_start( robot_model );

  robot_state_start.setVariablePositions( joint_values );
  robot_state_start.update();

  ros::Rate loop_rate(10);

  int count = 1;

  std::cout << BOLDGREEN( ) << " set the robot state" << RESET() << std::endl;
  std::vector<double> joint_group_positions;
  do
  {
    // muovo il robot!
    moveit::core::robotStateToJointStateMsg(robot_state_start,start_state_msg);
    move_robot_to_start_state.publish(start_state_msg);
    ros::spinOnce();
    loop_rate.sleep();

    joint_group_positions = move_group.getCurrentJointValues();
    count++;
    if (count > 15){
      ROS_ERROR("necessarie piu' di 15 iterazioni per raggiungere la posizione");
      return false;
    }
  } while( ! std::equal(joint_group_positions.begin(),joint_group_positions.end(), joint_values.begin() ));

  return true;
}

bool allowPanelCollisions(ros::NodeHandle& nh, const std::vector<std::string>& links, const std::string& link, bool allow)
{

  robot_model_loader::RobotModelLoader robot_model_loader ( "robot_description" );
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  planning_scene::PlanningScenePtr planning_scene ( new planning_scene::PlanningScene( kinematic_model ) );
  collision_detection::AllowedCollisionMatrix acm = planning_scene->getAllowedCollisionMatrix();

  for ( auto l : links)
    acm.setEntry(l,link,allow);

  std::vector<std::string> names;
  acm.getAllEntryNames(names);

  for (auto n : names)
    std::cout<<n<<std::endl;

  moveit_msgs::AllowedCollisionMatrix acm_msg;

  acm.getMessage(acm_msg);

  ros::Publisher planning_scene_diff_publisher = nh.advertise<moveit_msgs::PlanningScene> ( "planning_scene", 1 );

  while ( planning_scene_diff_publisher.getNumSubscribers() < 1 )
  {
      ros::WallDuration sleep_t ( 0.5 );
      sleep_t.sleep();
  }

  moveit_msgs::PlanningScene planning_scene_msg;

  planning_scene_msg.is_diff = true;
  planning_scene_msg.allowed_collision_matrix = acm_msg;

  planning_scene_diff_publisher.publish ( planning_scene_msg );
  ros::Duration ( 1.0 ).sleep();

  getPlanningScene ( nh, planning_scene );

  return true;

}

bool manageCollisions( ros::NodeHandle&                        nh
                      , const robot_model::RobotModelConstPtr&  robot_model
                      , const std::vector<std::string>&         links1
                      , const std::vector<std::string>&         links2
                      , bool                                    allow
                      , bool                                    verbose )
{
  planning_scene::PlanningScenePtr planning_scene( new planning_scene::PlanningScene( robot_model ) );
  
  ros::Publisher planning_scene_diff_publisher = nh.advertise<moveit_msgs::PlanningScene> ( "planning_scene", 1 );
  
  while ( planning_scene_diff_publisher.getNumSubscribers() < 1 )
  {
      ros::WallDuration sleep_t ( 0.5 );
      sleep_t.sleep();
  }

  ros::Time st = ros::Time::now();
  getPlanningScene ( nh, planning_scene );
  
  
  collision_detection::AllowedCollisionMatrix acm = planning_scene->getAllowedCollisionMatrix();
  for ( auto l1 : links1)
  {
    for ( auto l2 : links2)
    {
      if(verbose)
        std::cout << "l1: " << l1 << "l2: " << l2 << ( allow ? " DISABLE COLLISION " : " ENABLE COLLISION " ) << std::endl;
      acm.setEntry(l2,l1,allow);
    }
  }

  moveit_msgs::AllowedCollisionMatrix acm_msg;
  acm.getMessage(acm_msg);
  
  moveit_msgs::PlanningScene planning_scene_msg;

  planning_scene_msg.is_diff = true;
  planning_scene_msg.allowed_collision_matrix = acm_msg;

  do
  {   
    planning_scene_diff_publisher.publish ( planning_scene_msg );
    
    getPlanningScene( nh, planning_scene);
    
    collision_detection::AllowedCollisionMatrix nacm = planning_scene->getAllowedCollisionMatrix();
    if(verbose )
      std::cout << to_string( nacm ) << std::endl;
    bool ok = true;
    for ( auto l1 : links1)
    {
      for ( auto l2 : links2)
      {
        collision_detection::AllowedCollision::Type allowed_collision_type;
        if( nacm.getEntry(l1,l2,allowed_collision_type) )
        {
          ok &= (allowed_collision_type == ( allow ? collision_detection::AllowedCollision::ALWAYS : collision_detection::AllowedCollision::NEVER ) );
        }
      }
    }
    if( ok ) 
    {
      break;
    }
    
    if( (ros::Time::now() - st).toSec() < 10.0)
    {
      ROS_FATAL("Timeout expired. Return false");
      return false;
    }
  } while( ros::ok() );

  return true;

}

std::string to_string( const collision_detection::AllowedCollisionMatrix& acm )
{
  std::string ret;
  std::vector<std::string> names;
  acm.getAllEntryNames(names);
  auto const & it = std::max_element( names.begin(), names.end(), [&]( const std::string& s1,const std::string& s2 ){ return s1.length() < s2.length(); } );
  assert( it != names.end() );
  size_t mx = it->length() + 8;
  std::string header = "Names";
  header.append(mx - header.length(), '-');
  
  for( int i=0;i<names.size();i++ ) 
  {
    std::string is = std::to_string(i);
    header += "#" + is + ( (is.length() == 1 ) ?  " " : "" );
  }
  
  ret = header + "\n";
  for( int i=0;i<names.size();i++ ) 
  {
    std::string name = "#" + std::to_string(i) + names.at(i);
    name.append( mx - name.length(), ' ');
    std::string row = name + ":";
    for( const auto & nn : names ) 
    {
      collision_detection::AllowedCollision::Type t;
      if(!acm.getEntry(names.at(i), nn, t) )
      {
        row +=  BOLDYELLOW( ) +" ? " + RESET();
      }
      else
        row += (t == collision_detection::AllowedCollision::ALWAYS ? BOLDGREEN() + " Y " + RESET() : BOLDRED() + " N " + RESET()  );
    }
      
    ret += row +"\n";
  }

  return ret;
}


}
