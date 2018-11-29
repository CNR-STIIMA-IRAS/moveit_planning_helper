#ifndef __ITIA_MVUTILS__H__
#define __ITIA_MVUTILS__H__

#include <ros/ros.h>
#include <moveit/collision_detection/collision_common.h>

#if ROS_VERSION_MINIMUM(1, 12, 0) //  kinetic
    #include <moveit/move_group_interface/move_group_interface.h>
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

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

namespace moveit_planning_helper
{
  
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
void getPlanningScene ( ros::NodeHandle&                            nh
                      , const moveit::core::RobotModelConstPtr      robot_model
                      , const std::string&                          ns
                      , planning_scene::PlanningScenePtr            ret);

/**
 * @fn setRobotStateNH
 * 
 */
bool setRobotStateNH( ros::NodeHandle&                          nh
                    , const moveit::core::RobotState&           robot_state
                    , const moveit::core::RobotModelConstPtr    robot_model
                    , const std::string&                        group_name
                    , const std::string&                        ns    );

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
