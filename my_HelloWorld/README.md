# First Program in ROS
After understanding ROS concepts, we are ready to code. In this example, we will create our first **ROS pkg** with 2 nodes that communicate each other with **ROS topics**. Before that, we have to do some preliminary steps.

## 0) Prerequisities:
- Ubuntu 20.04 LTS.
- ROS Noetic.

## 1) Create your workspace
We are using `catkin` build system to build ROS pkgs. A build system is responsible for generating `targets` (executable/libraries) from a source code. The first requirement to work with ROS pkgs is to create a ROS `catkin` workspace. Type in the command:
```
mkdir -p ~/catkin_ws/src
```
This command create a `catkin_ws` directory, with an inner folder `src`. In order to get access to ROS functions, we have to source the ROS workspace.

```
source /opt/ros/noetic/setup.bash
```
If you followed this [link](http://wiki.ros.org/noetic/Installation/Ubuntu) you already done this step. So let's switch to  `src` folder, typing:
```
cd ~/catkin_ws/src
```
Finally, initialize a new `catkin` workspace:
```
catkin_init_workspace
```

## 2) Build your new workspace

