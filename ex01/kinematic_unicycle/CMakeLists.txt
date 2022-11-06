cmake_minimum_required(VERSION 3.0.2)
project(kinematic_unicycle)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
)

catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

# This will create executable of the node
add_executable(model src/model.cpp)
# This will link executable to the appropriate libraries
target_link_libraries(model ${catkin_LIBRARIES})