cmake_minimum_required(VERSION 3.0.2)
project(hello_world_pkg)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

##### Insert your Nodes here

## PUBLISHER
# This will create executable of the node
add_executable(publisher src/publisher.cpp)
# This will link executable to the appropriate libraries
target_link_libraries(publisher ${catkin_LIBRARIES})

## SUBSCRIBER
# This will create executable of the node
add_executable(subscriber src/subscriber.cpp)
# This will link executable to the appropriate libraries
target_link_libraries(subscriber ${catkin_LIBRARIES})