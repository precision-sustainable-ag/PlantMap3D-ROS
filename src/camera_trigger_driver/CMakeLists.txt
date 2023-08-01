cmake_minimum_required(VERSION 2.8.3)
project(camera_trigger_driver)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  message_generation
)



## Generate messages in the 'msg' folder
add_message_files(
  FILES
  PM3DGPSData.msg
 )

## Generate services in the 'srv' folder
add_service_files(
  FILES
   PM3DGPSHeading.srv
 )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   sensor_msgs#   std_msgs
 )


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES camera_trigger
  CATKIN_DEPENDS message_runtime roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
  include/geotoutm
  ${catkin_INCLUDE_DIRS}
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(camera_trigger_driver src/camera_trigger_node.cpp
				    include/geotoutm/LatLong-UTMconversion.cpp
            include/geotoutm/computeGPSDistance.cpp)


add_dependencies(camera_trigger_driver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
 target_link_libraries(camera_trigger_driver
   ${catkin_LIBRARIES}
 )

