cmake_minimum_required(VERSION 2.8.3)
project(moveit_planning_helper)

set(CMAKE_BUILD_TYPE Debug)
add_definitions( -std=c++11 -funroll-loops -O3 )

find_package(catkin REQUIRED COMPONENTS
  moveit_core
  roscpp
  moveit_ros_planning
  moveit_ros_planning_interface
  eigen_matrix_utils
  actionlib
  moveit_visual_tools
  actionlib_msgs
  subscription_notifier
  name_sorting
)

add_action_files(
  FILES
  ExecuteTrajectoryFromParam.action
)
generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES moveit_planning_helper
 CATKIN_DEPENDS moveit_core roscpp eigen_matrix_utils moveit_ros_planning  moveit_ros_planning_interface actionlib moveit_visual_tools actionlib_msgs name_sorting
 DEPENDS 
)


include_directories(
include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/iterative_time_parameterization.cpp
  src/${PROJECT_NAME}/manage_trajectories.cpp
  src/${PROJECT_NAME}/multiplugins_planner.cpp
  src/${PROJECT_NAME}/spline_interpolator.cpp
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_executable(execute_trj_move_group src/execute_trj_moveit.cpp)
add_dependencies(execute_trj_move_group ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(execute_trj_move_group
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(execute_trj_service src/execute_trj_service.cpp)
add_dependencies(execute_trj_service ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(execute_trj_service
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(ptp_action_server src/ptp_action_server.cpp)
add_dependencies(ptp_action_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ptp_action_server
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
add_executable(execute_trj_from_param src/execute_trj_from_param.cpp)
add_dependencies(execute_trj_from_param ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(execute_trj_from_param
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(override_manager src/override_manager.cpp)
add_dependencies(override_manager ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(override_manager
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
add_executable(execute_trj_cartesian_from_msg src/execute_trj_cartesian_from_msg.cpp)
add_dependencies(execute_trj_cartesian_from_msg ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(execute_trj_cartesian_from_msg
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(test_joint_multiquery src/test_joint_multiquery.cpp)
add_dependencies(test_joint_multiquery ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_joint_multiquery
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)


install(TARGETS ${PROJECT_NAME} execute_trj_move_group execute_trj_service execute_trj_from_param override_manager
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

