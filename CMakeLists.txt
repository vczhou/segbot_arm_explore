cmake_minimum_required(VERSION 2.8.3)
project(segbot_arm_explore)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  message_generation
  jaco_msgs
  actionlib
  pcl_ros
  pcl_conversions
  agile_grasp
  tf_conversions
  moveit_ros_planning_interface
  moveit_utils
  moveit_msgs
  nav_msgs
  sound_play
  move_base_msgs
  segbot_arm_perception
)

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

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

#catkin_package(
#  INCLUDE_DIRS include
#  DEPENDS message_runtime segbot_arm_perception actionlib_msgs geometry_msgs std_msgs
#)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(demo_grasp_explore src/demo_grasp_explore.cpp)
target_link_libraries(demo_grasp_explore ${catkin_LIBRARIES} )
add_dependencies(demo_grasp_explore segbot_arm_manipulation_gencpp)

#install(DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#  FILES_MATCHING PATTERN "*.h"
#  PATTERN ".svn" EXCLUDE)
#
