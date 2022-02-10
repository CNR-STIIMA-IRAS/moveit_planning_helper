#!/bin/bash

source "${ICI_SRC_PATH}/workspace.sh"
source "${ICI_SRC_PATH}/util.sh"

echo "Generating coverage for 'moveit_planning_helper'"

ws=~/target_ws
extend="/opt/ros/$ROS_DISTRO"
ici_exec_in_workspace "$extend" "$ws" catkin build moveit_planning_helper -v --no-deps --catkin-make-args coverage_report

echo "Uploading coverage results to codecov.io"

# Remove duplicated information
rm "$ws/build/moveit_planning_helper/coverage_report.info.cleaned"
rm "$ws/build/moveit_planning_helper/coverage_report.info.removed"


# Actually upload coverage informationstate_space_controllers
bash <(curl -s https://codecov.io/bash) -s "$ws/build/moveit_planning_helper/"
