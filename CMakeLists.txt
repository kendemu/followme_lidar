cmake_minimum_required(VERSION 2.8.3)
project(dfollow2)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs urg_node sensor_msgs joy OpenCV)

## add demu
include_directories(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
include_directories(/usr/local/include)

link_directories(/usr/local/lib)


catkin_package(
  INCLUDE_DIRS
  LIBRARIES dfollow2
  CATKIN_DEPENDS roscpp geometry_msgs
  DEPENDS urg_node sensor_msgs roscpp
)

###########
## Build ##
###########

add_executable(dfollow2 src/dfollow2.cpp)
target_link_libraries(dfollow2 ${catkin_LIBRARIES} libkobuki.so ${OpenCV_LIBRARIES} )

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
#install(PROGRAMS
#  scripts/move
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

## Mark executables and/or libraries for installation
install(TARGETS dfollow2
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark all other useful stuff for installation
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
