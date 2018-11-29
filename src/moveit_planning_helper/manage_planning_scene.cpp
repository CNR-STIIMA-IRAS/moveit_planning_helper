#include <moveit/move_group/capability_names.h>
#include <eigen_conversions/eigen_msg.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit_planning_helper/manage_planning_scene.h>
#include <moveit_planning_helper/conversions.h>
#include <numeric>
static const char* DEFAULT      = "\033[0m";
static const char* RESET        = "\033[0m";
static const char* BLACK        = "\033[30m";
static const char* RED          = "\033[31m";
static const char* GREEN        = "\033[32m";
static const char* YELLOW       = "\033[33m";
static const char* BLUE         = "\033[34m";
static const char* MAGENTA      = "\033[35m";
static const char* CYAN         = "\033[36m";
static const char* WHITE        = "\033[37m";
static const char* BOLDBLACK    = "\033[1m\033[30m";
static const char* BOLDRED      = "\033[1m\033[31m";
static const char* BOLDGREEN    = "\033[1m\033[32m";
static const char* BOLDYELLOW   = "\033[1m\033[33m";
static const char* BOLDBLUE     = "\033[1m\033[34m";
static const char* BOLDMAGENTA  = "\033[1m\033[35m";
static const char* BOLDCYAN     = "\033[1m\033[36m";
static const char* BOLDWHITE    = "\033[1m\033[37m";


namespace moveit_planning_helper
{
  

template< typename T > std::string to_string  ( const std::vector< T >& v
                                              , const std::string prefix = "["
                                              , const std::string delimeter = ", "
                                              , const std::string terminator ="]" )
{
  std::string ret = prefix;
  if(v.size() == 0)
    return "";
  
  for( size_t i=0; i < v.size()-1; i++)
    ret += std::to_string( v[i] ) + delimeter;
  ret += std::to_string( v.back() ) + terminator;
  
  return ret;
}

  
template<> inline std::string to_string<>( const std::vector< std::string >& v
                              , const std::string prefix
                              , const std::string delimeter
                              , const std::string terminator )
{
  
  std::string ret = prefix;
  if( v.size() > 0 )
  { 
    for( size_t i=0; i < v.size()-1; i++)
      ret += v[i] + delimeter;
    ret += v.back() + terminator;
  }
  else
    ret += terminator;
  
  return ret;
}
template <typename T>
std::string to_string_with_precision(const T a_value, const int n = 6)
{
    std::ostringstream out;
    out << std::fixed << std::setprecision(n) << a_value;
    return out.str();
}

template< typename T > std::string to_string_keys ( const std::map< std::string, T >& m
                                                  , const std::string prefix = "["
                                                  , const std::string delimeter = ", "
                                                  , const std::string terminator ="]" )
{
  std::string ret = prefix;
  if(m.size() == 0)
    return "";
  
  for( auto im = m.begin(); im != m.end(); im++)
    ret += im->first + delimeter;
  ret += terminator;
  
  return ret;
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


std::string to_string (const geometry_msgs::Pose& p )
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
                      , const moveit::core::RobotModelConstPtr robot_model
                      , const std::string& ns
                      , planning_scene::PlanningScenePtr ret)
{
  ros::ServiceClient planning_scene_service;
  planning_scene_service = nh.serviceClient<moveit_msgs::GetPlanningScene>(ns + "/" +move_group::GET_PLANNING_SCENE_SERVICE_NAME);
  if(!planning_scene_service.waitForExistence( ros::Duration(5.0) ) )
  {
    ROS_ERROR("getPlanningScene Failed: service '%s' does not exist ", (ns + "/" +move_group::GET_PLANNING_SCENE_SERVICE_NAME).c_str() );
  }
    
  moveit_msgs::PlanningScene planning_scene_msgs;
  
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
  
  ret->setPlanningSceneMsg(planning_scene_msgs);
  
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
    getPlanningScene(nh, robot_model, ns, planning_scene );
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

  if(verbose) ROS_INFO_STREAM( BOLDYELLOW << "Configuration: \n" << RESET << pose );   

  std::vector< std::vector<double> >  q;
  kinematics::KinematicsResult        result;
  kinematics::KinematicsQueryOptions  options;
  
  std::vector<double> ik_seed_state = first_tentative_ik_seed_state;
  
  assert( ikine_solver );
  std::cout << ikine_solver.get() << std::endl;
  if(verbose) ROS_INFO_STREAM( BOLDYELLOW << "Get Position: \n" << RESET << pose );   
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
        ROS_INFO_STREAM( BOLDYELLOW << i++ <<"/" << q.size() << "# Solution: \n" << RESET << ret );     
      }
    }
    
    return true;
  }
  
  if(verbose) ROS_WARN_STREAM( BOLDYELLOW << "no ik solutions" );   

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
    ROS_INFO(" %sIn collision ", BOLDRED );
    collision_detection::CollisionResult::ContactMap::const_iterator it;
    for(it = collision_result.contacts.begin(); it != collision_result.contacts.end(); ++it)
      ROS_INFO(" - Contact between: %s and %s", it->first.first.c_str(), it->first.second.c_str());
  }
  else if ( verbose )
    ROS_INFO("%s Not in collision ", BOLDGREEN );

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
      ROS_ERROR_STREAM( BOLDRED << "Fkine failed:" );
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
      ROS_ERROR_STREAM( BOLDRED << "Fkine failed:" );
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
    ROS_ERROR_STREAM( BOLDRED << "Collision Check failed." );
    return false;
  }
  
  if(verbose)
  {
    geometry_msgs::Pose ee_msgs = moveit_planning_helper::robotStateToPoseMsg(robot_state_.front(), last_link_);  
    ROS_INFO_STREAM( BOLDYELLOW << "Pose: " << RESET << to_string( ee_msgs) ); 
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
  getPlanningScene( nh_, robot_model_, ns_, planning_scene);
  
  bool ok = setRobotState( joints_positions, planning_scene, calc_equivalent_joint_configurations, verbose);
  
  return ok; 
}


bool RobotStateExtended::setRobotState  ( const robot_state::RobotState& robot_state
                                        , bool                           calc_equivalent_joint_configurations 
                                        , bool                           verbose )
{
  setRobotStateNH(nh_, robot_state, robot_model_, kin_group_ , ns_ );  
  
  planning_scene::PlanningScenePtr planning_scene( new planning_scene::PlanningScene( robot_model_ ) );
  getPlanningScene( nh_, robot_model_, ns_, planning_scene );
  
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


}
