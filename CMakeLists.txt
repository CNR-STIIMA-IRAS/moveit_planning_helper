cmake_minimum_required(VERSION 2.8.3)

project(moveit_planning_helper)

###################
## PRELIMINARIES ##
###################
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

IF("${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}" LESS 3.0)
  set(CMAKE_CXX_FLAGS "-std=c++11")
ELSE()
  # Default to C++14
  if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
  endif()

  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

option(ENABLE_TESTING "ENABLE TESTING" OFF)
option(ENABLE_COVERAGE_TESTING "ENABLE COVERAGE TESTING" OFF)
option(COMPILE_EXAMPLE "COMPILE THE EXAMPLE" OFF)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  eigen_matrix_utils
  actionlib
  moveit_visual_tools
  actionlib_msgs
  control_msgs
  subscription_notifier
  name_sorting
  rosparam_utilities
)

###########
set(ENABLE_TESTING ${CATKIN_ENABLE_TESTING})
if(ENABLE_TESTING)
  find_package(rostest REQUIRED)

  if(ENABLE_COVERAGE_TESTING)
    find_package(code_coverage REQUIRED)
    APPEND_COVERAGE_COMPILER_FLAGS()
  endif(ENABLE_COVERAGE_TESTING)
endif(ENABLE_TESTING)
###########

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
  src/${PROJECT_NAME}/manage_trajectories.cpp
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

add_executable(test_planning_feasibility src/test_planning_feasibility.cpp)
add_dependencies(test_planning_feasibility ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_planning_feasibility
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)


install(TARGETS ${PROJECT_NAME} execute_trj_move_group execute_trj_service execute_trj_from_param override_manager test_planning_feasibility
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

if(ENABLE_TESTING)

    add_rostest_gtest(manage_trajectory_test test/manage_trajectory_test.launch test/manage_trajectory_test.cpp)
    target_link_libraries(manage_trajectory_test ${PROJECT_NAME} ${catkin_LIBRARIES} ${rostest_LIBRARIES} )

    if( ENABLE_TESTING AND ENABLE_COVERAGE_TESTING)
    message(STATUS "The coverage testing is enabled")
      set(COVERAGE_EXCLUDES "*/${PROJECT_NAME}/test*")
      add_code_coverage(
        NAME coverage_report
        DEPENDENCIES ${PROJECT_NAME}_test
      )
    endif()

endif(ENABLE_TESTING)
