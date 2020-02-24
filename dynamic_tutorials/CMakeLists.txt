cmake_minimum_required(VERSION 2.8.3)
project(dynamic_tutorials)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  roscpp
  rospy
)

#add dynamic reconfigure api
#find_package(catkin REQUIRED dynamic_reconfigure)
generate_dynamic_reconfigure_options(
  cfg/Tutorials.cfg
  #...
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES dynamic_tutorials
#  CATKIN_DEPENDS dynamic_reconfigure roscpp rospy
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


