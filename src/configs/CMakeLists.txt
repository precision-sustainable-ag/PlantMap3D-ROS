cmake_minimum_required(VERSION 3.0.2)
project(configs)

find_package(catkin REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES configs
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
# include
# ${catkin_INCLUDE_DIRS}
)


install(DIRECTORY config/
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)