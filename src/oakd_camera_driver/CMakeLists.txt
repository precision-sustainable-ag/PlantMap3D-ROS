cmake_minimum_required(VERSION 3.0.2)
project(oakd_camera_driver)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  rospy
  sensor_msgs
  message_generation
  camera_trigger_driver
)



## Generate messages in the 'msg' folder
add_message_files(
   FILES
   PM3DCameraData.msg
   # PM3DGPSData.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   sensor_msgs
   camera_trigger_driver
 )


catkin_package(
	CATKIN_DEPENDS rospy message_runtime camera_trigger_driver
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES oakd_camera_driver
#  CATKIN_DEPENDS cv_bridge rospy sensor_msgs
#  DEPENDS system_lib
)


include_directories(
include
  ${catkin_INCLUDE_DIRS}
)

