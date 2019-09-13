#ifndef __ITIA_MVUTILS__H__
#define __ITIA_MVUTILS__H__

#include <eigen3/Eigen/Core>
#include <ros/ros.h>
#include <tf/tf.h>
#include <moveit/collision_detection/collision_common.h>

#if ROS_VERSION_MINIMUM(1, 12, 0) //  kinetic
    #include <moveit/move_group_interface/move_group_interface.h>
    #include <moveit/planning_scene_interface/planning_scene_interface.h>
#else
    #include <moveit/move_group_interface/move_group.h>
    namespace moveit
    {
        namespace planning_interface
  {
          typedef ::moveit::planning_interface::MoveGroup MoveGroupInterface;
  }
    }
#endif

#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit_msgs/ApplyPlanningScene.h>
#include <moveit_msgs/ObjectColor.h>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <descartes_moveit/ikfast_moveit_state_adapter.h>
#include <descartes_trajectory/axial_symmetric_pt.h>
#include <descartes_trajectory/cart_trajectory_pt.h>
#include <descartes_trajectory/joint_trajectory_pt.h>
#include <descartes_planner/dense_planner.h>
#include <descartes_utilities/ros_conversions.h>
#include <cartesian_path_utils/cartesian_path_utils.h>


namespace moveit_planning_helper
{

  inline const std::map< int, std::string >& getMoveitErrorCodesIds( )
  {
    static std::map< int, std::string > ret {   {   1,      "SUCCESS                                      "}
                                            ,   {  99999,   "FAILURE                                      "}
                                            ,   {  -1,      "PLANNING_FAILED                              "}
                                            ,   {  -2,      "INVALID_MOTION_PLAN                          "}
                                            ,   {  -3,      "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE"}
                                            ,   {  -4,      "CONTROL_FAILED                               "}
                                            ,   {  -5,      "UNABLE_TO_AQUIRE_SENSOR_DATA                 "}
                                            ,   {  -6,      "TIMED_OUT                                    "}
                                            ,   {  -7,      "PREEMPTED                                    "}
                                            ,   {  -10,     "START_STATE_IN_COLLISION                     "}
                                            ,   {  -11,     "START_STATE_VIOLATES_PATH_CONSTRAINTS        "}
                                            ,   {  -12,     "GOAL_IN_COLLISION                            "}
                                            ,   {  -13,     "GOAL_VIOLATES_PATH_CONSTRAINTS               "}
                                            ,   {  -14,     "GOAL_CONSTRAINTS_VIOLATED                    "}
                                            ,   {  -15,     "INVALID_GROUP_NAME                           "}
                                            ,   {  -16,     "INVALID_GOAL_CONSTRAINTS                     "}
                                            ,   {  -17,     "INVALID_ROBOT_STATE                          "}
                                            ,   {  -18,     "INVALID_LINK_NAME                            "}
                                            ,   {  -19,     "INVALID_OBJECT_NAME                          "}
                                            ,   {  -21,     "FRAME_TRANSFORM_FAILURE                      "}
                                            ,   {  -22,     "COLLISION_CHECKING_UNAVAILABLE               "}
                                            ,   {  -23,     "ROBOT_STATE_STALE                            "}
                                            ,   {  -24,     "SENSOR_INFO_STALE                            "}
                                            ,   {  -31,     "NO_IK_SOLUTION                               "}
                                            };
    return ret;
  }


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




std::ostream& operator<<(std::ostream& stream, const Eigen::Affine3d& affine);
std::ostream& operator<<(std::ostream& stream, const tf::Transform& transform);
std::string to_string(const tf::Pose& pose);
std::string to_string(const geometry_msgs::Pose&  pose);
std::string to_string(const Eigen::Affine3d&      pose);
std::string to_string(const Eigen::Isometry3d&    pose);



std::string to_string( const collision_detection::AllowedCollisionMatrix& acm );
/**
 * @fn getJointPositions
 * @param const moveit::core::RobotState& robot_state
 * @param const moveit::core::JointModelGroup*  joint_model_group
 * @return std::map<std::string, std::vector< double> > 
 */
std::map<std::string, std::vector< double> > getJointPositions( const moveit::core::RobotState& robot_state, const moveit::core::JointModelGroup*  joint_model_group );

/**
 * @fn normFrobenius
 * 
 */
double normFrobenius( const std::map<std::string, std::vector< double> >& robot_state1, const std::map<std::string, std::vector< double> >& robot_state2 );


/**
 * @fn rvizDisplayPath
 */
void rvizDisplayPath(ros::NodeHandle& nh, const moveit::planning_interface::MoveGroupInterface::Plan& plan, const std::string& ns = "/move_group/"  );

/**
 * @fn getParam
 * 
 */
bool getParam( const ros::NodeHandle&   nh
             , std::string              name_param
             , geometry_msgs::Pose&     pose
             , std::string              param = "pose-geometry-msg" );

/**
 * @fn getRobotStateNH
 * 
 */
robot_state::RobotState getRobotState( ros::NodeHandle&                         nh
                                     , const moveit::core::RobotModelConstPtr   robot_model
                                     , const std::string&                       ns = "/move_group/" ) ;

/**
 * @fn getPlanningScene
 * 
 */
void getPlanningScene ( ros::NodeHandle&              nh
                      , const std::string&            ns
                      , moveit_msgs::PlanningScene&   planning_scene_msgs);

void getPlanningScene ( ros::NodeHandle&                  nh
                      , planning_scene::PlanningScenePtr& ret );

/**
 * @fn setRobotStateNH
 * 
 */
bool setRobotStateNH( ros::NodeHandle&                          nh
                    , const moveit::core::RobotState&           robot_state
                    , const moveit::core::RobotModelConstPtr    robot_model
                    , const std::string&                        group_name
                    , const std::string&                        ns    );


bool setRobotState(ros::NodeHandle& nh, const std::vector<double> &joint_values , moveit::planning_interface::MoveGroupInterface &move_group);

std::shared_ptr< moveit_msgs::CollisionObject > toCollisionObject ( const std::string&          collisionObjID
                                                                  , const std::string&          path_to_mesh
                                                                  , const std::string&          reference_frame
                                                                  , const tf::Pose&             pose
                                                                  , const Eigen::Vector3d       scale = { 1.0, 1.0, 1.0} );

std::shared_ptr< moveit_msgs::CollisionObject > toCollisionObject ( const std::string&          collisionObjID
                                                                  , const std::string&          path_to_mesh
                                                                  , const std::string&          reference_frame
                                                                  , const geometry_msgs::Pose&  pose 
                                                                  , const Eigen::Vector3d       scale = { 1.0, 1.0, 1.0} );

std::shared_ptr< moveit_msgs::CollisionObject > toCollisionObject ( const std::string&          collisionObjID
                                                                  , const std::string&          path_to_mesh
                                                                  , const std::string&          reference_frame
                                                                  , const Eigen::Affine3d&      pose 
                                                                  , const Eigen::Vector3d       scale = { 1.0, 1.0, 1.0} );


bool applyAndCheckPS  (ros::NodeHandle nh
                      , std::vector<moveit_msgs::CollisionObject> cov
                      , std::vector<moveit_msgs::ObjectColor> colors
                      , ros::Duration  timeout);


void vecToTf ( std::vector<double> *pose ,tf::Pose& transform );


bool plan   ( const geometry_msgs::Pose                             &target_pose
            , const std::string                                     &planning_goup
            , moveit::core::RobotModelPtr                           &robot_model
            , moveit::planning_interface::MoveGroupInterface        &move_group
            , std::vector<double>                                   &target_jpos
            , moveit::planning_interface::MoveGroupInterface::Plan  &my_plan);




bool plan ( const std::vector<double>&                              target_jpos
          , moveit::planning_interface::MoveGroupInterface&         move_group
          , moveit::planning_interface::MoveGroupInterface::Plan&   my_plan );


bool cartesian( const geometry_msgs::Pose                             &starting_pose
              , const geometry_msgs::Pose                             &target_pose
              , const tf::StampedTransform                            &worldToRobot
              , const double                                          &default_velocity
              , moveit::planning_interface::MoveGroupInterface        &move_group
              , moveit::planning_interface::MoveGroupInterface::Plan  &my_plan );



bool descartes( const geometry_msgs::Pose                                     &starting_pose
              , const geometry_msgs::Pose                                     &target_pose
              , const std::vector<std::string>                                &joint_names
              , boost::shared_ptr<descartes_moveit::IkFastMoveitStateAdapter> &robot_model
              , descartes_planner::DensePlanner                               &planner
              , moveit::planning_interface::MoveGroupInterface::Plan& my_plan
              , std::vector<double>   starting_point = std::vector<double>());

bool allowPanelCollisions(ros::NodeHandle& nh, const std::vector<std::string>& links, const std::string& link, bool allow = true );

bool manageCollisions ( ros::NodeHandle& nh
                      , const robot_model::RobotModelConstPtr& robot_model
                      , const std::vector<std::string>& links1
                      , const std::vector<std::string>& links2
                      , bool allow = true
                      , bool verbose = false );

bool  attachObj( const std::string ns, std::string object_id, std::string frame_id, std::string link, bool attach = true);

/**
 * @class RobotStateExtended
 * 
 * 
 */

class RobotStateExtended {
  
    ros::NodeHandle                         nh_;
    const std::string                       ns_;
    robot_model::RobotModelConstPtr         robot_model_;
    const std::string                       kin_group_;
    const std::string                       last_link_;
    std::vector< robot_state::RobotState >  robot_state_;
    std::vector< std::vector<double> >      robot_state_joints_;
    std::vector<std::string>                robot_state_joints_names_;
    const moveit::core::JointModelGroup*    joint_model_group_; 
    
    
public:
  
  /**
   * Constructor
   * @param nh Node handle, necessary to call internally getParam()
   * @param robot_model Stored in the class, not modifiable by any other function
   * @param kin_group name of the kinematic planning group
   * @param last_lin name of the last link frame
   */
    RobotStateExtended( const ros::NodeHandle&                    nh
                      , robot_model::RobotModelConstPtr           robot_model
                      , const std::string&                        kin_group
                      , const std::string&                        last_link
                      , const std::string&                        ns );
    
    const std::vector< robot_state::RobotState >& getRobotStateConfigurations( ) const;
    const std::vector< std::vector<double> >&     getRobotStateJointConfigurations( ) const;
    const std::vector< std::string >&             getRobotStateJointsNames( ) const;

    
    /**
     * @param pose
     * @param solutions all the possibile results given from the inverse kinematics
     * @param first_tentative_ik_seed_state seed for the ik
     * @param verbose
     */

    bool ikine( const kinematics::KinematicsBaseConstPtr&  ikine_solver // = robot_model->getJointModelGroup(kin_group)->getSolverInstance()
              , const geometry_msgs::Pose&                 pose
              , std::vector <std::vector <double> >*       solutions
              , const std::vector<double>&                 first_tentative_ik_seed_state = std::vector<double>()
              , const bool                                 verbose = true ) const;
    
    bool fkine( const kinematics::KinematicsBaseConstPtr&  ikine_solver 
              , const std::vector<double>&                 joints
              , geometry_msgs::Pose*                       pose
              , std::vector <std::vector <double> >*       equivalent_solutions
              , const bool                                 verbose = true ) const;
    
    bool fkine( const kinematics::KinematicsBaseConstPtr&  ikine_solver 
              , const std::map<std::string,double>&        joints
              , geometry_msgs::Pose*                       pose
              , std::vector <std::vector <double> >*       equivalent_solutions
              , const bool                                 verbose = true ) const;
   
    /**
     * 
     */
    bool setRobotState  ( const std::vector<double>&        joints
                        , planning_scene::PlanningScenePtr  planning_scene
                        , bool                              calc_equivalent_joint_configurations  = false
                        , bool                              verbose                               = false);

    /**
     * look the robot state in the ros param
     * @param ros_param
     * @param planning_scene
     * @param 
     */
    bool setRobotState  ( const robot_state::RobotState&    robot_state
                        , planning_scene::PlanningScenePtr  planning_scene
                        , bool                              calc_equivalent_joint_configurations  = false
                        , bool                              verbose                               = false);
  
    /**
     * check the collision of the actual robot state, without caring to update rviz and move_group node itnernal state
     */
    bool checkRobotState( planning_scene::PlanningScenePtr  planning_scene
                        , bool                              calc_equivalent_joint_configurations  = false
                        , bool                              verbose                               = false);
    /**
     * check the collision, without caring to update rviz and move_group node itnernal state
     */
    bool checkCollision ( const std::vector<double>&        joints
                        , planning_scene::PlanningScenePtr  planning_scene
                        , bool                              verbose = true );

    
    //-------------------------------------------------//
    // update rviz and move_group node itnernal state  
    //-------------------------------------------------//
    /**
     * 
     */
    bool setRobotState  ( const std::vector<double>&        joints_positions
                        , bool                              calc_equivalent_joint_configurations = false
                        , bool                              verbose                              = false);
    
    bool setRobotState  ( const robot_state::RobotState&    robot_state
                        , bool                              calc_equivalent_joint_configurations  = false
                        , bool                              verbose                               = false);
    
    bool checkRobotState( bool                              calc_equivalent_joint_configurations  = false
                        , bool                              verbose                               = false);

    /**
     * 
     * 
     */
    bool updateTF ( int pub_time, const int idx_configuration = 0 );

};

typedef std::shared_ptr<RobotStateExtended> RobotStateExtendedPtr;

}


#endif
