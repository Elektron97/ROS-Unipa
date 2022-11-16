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
To build you workspace, type in the terminal:
```
catkin_make
```
This last command will create a `devel` and `build` directory in the `catkin_ws`. To add the configurated ROS workspace to the ROS environment, type:

```
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
## 3) Create yor First ROS pkg
Now that the workspace is configured, we can create a new pkg. To do that, switch in the `src` folder (`cd ~/catkin_ws/src`) and type this command, substituing `package_name` with your package name and the dependencies.
```
catkin_create_pkg package_name [dependency1] [dependency2]
```

In our case, type:
```
catkin_create_pkg hello_world_pkg roscpp std_msgs
```
The dependencies are:
- `roscpp`: This is the C++ implementatioin of ROS. This client library implements ROS topics, services and so on.
- `std_msgs`: This package contains the basic definition of ROS primitive data types, such as integer, float, string, etc.

After package creation, your `hello_world_pkg` folder, you can write your nodes in `src` folder.

## 4) Publisher Node:
You can find the entire node [here](hello_world_pkg/src/publisher.cpp). This node publish a `string` (*"Hello World"*) at a specific frequency, `NODE_FREQUENCY`. Now let's break the code down.

- `Include` part: In this part we have to include the libraries used in our node.
 ```
 #include "ros/ros.h"
 #include "std_msgs/String.h"
 ```
First library includes the ROS client library, useful for the creation of a node, communications, parameters and so on. The second one imports the primitive type of msgs `string`. 
- After create the `main` with the arguments (`argc`, `argv`), we can init the node.
```
ros::init(argc, argv, "talker");
```
Here, the name chosen for our node is `talker`. 
- We have to also define the `NodeHandle` object. This object allows us to publish, subscribe, get parameters and so on.
```
ros::NodeHandle node_obj;
```
- After this, we can create the pub object. 
```
ros::Publisher string_pub = node_obj.advertise<std_msgs::String>("string_topic", QUEUE_SIZE);
```
This object tells the master that we are going to be publishing a message of type `std_msgs/String` on the topic `string_topic`. The `QUEUE_SIZE` is the size of a publishing queue. If we are publishing to quickly, it will buffer up a maximum of `QUEUE_SIZE` messages.
