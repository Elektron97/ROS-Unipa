cmake_minimum_required(VERSION 3.0.2)
project(robot_2dof)

find_package(catkin REQUIRED COMPONENTS
  gazebo_msgs
  gazebo_plugins
  gazebo_ros
  gazebo_ros_control
  geometry_msgs
  roscpp
  sensor_msgs
)

catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
