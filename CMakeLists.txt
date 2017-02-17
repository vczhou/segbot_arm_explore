cmake_minimum_required(VERSION 2.8.3)
project(segbot_arm_explore)

find_package(catkin REQUIRED COMPONENTS
  jaco_msgs
  moveit_msgs
  roscpp
  rospy
  std_msgs
  actionlib_msgs
  actionlib
  moveit_ros_planning_interface
  moveit_utils
  moveit_msgs
  nav_msgs
  move_base_msgs
  segbot_arm_perception
  segbot_arm_manipulation
  pcl_ros
  pcl_conversions
  bwi_msgs
)

#find_package(PCL 1.7 REQUIRED)

#include_directories(${PCL_INCLUDE_DIRS})
#link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})

#add_action_files(
#   FILES
#   TabletopGrasp.action
#   TabletopApproach.action
#)

#add_service_files(
#   FILES
#   iSpyTouch.srv
#   iSpyDetectTouch.srv
#)

#generate_messages(
#  DEPENDENCIES
#  sensor_msgs
#  actionlib_msgs
#  geometry_msgs
#  std_msgs
#)

catkin_package(
#  INCLUDE_DIRS include
#  DEPENDS message_runtime segbot_arm_perception actionlib_msgs geometry_msgs std_msgs
  CATKIN_DEPENDS actionlib_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## executables
add_executable(demo_grasp_explore src/demo_grasp_explore.cpp)
target_link_libraries(demo_grasp_explore ${catkin_LIBRARIES} )
add_dependencies(demo_grasp_explore segbot_arm_manipulation_gencpp)

#install(DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#  FILES_MATCHING PATTERN "*.h"
#  PATTERN ".svn" EXCLUDE)
#
