cmake_minimum_required(VERSION 3.0.2)
project(biomass_estimator_driver)


find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)


## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

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
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES biomass_estimator_driver
#  CATKIN_DEPENDS rospy std_msgs
#  DEPENDS system_lib
)


include_directories(
include
  ${catkin_INCLUDE_DIRS}
)

catkin_add_nosetests(test/test_biomass_estimator.py)

