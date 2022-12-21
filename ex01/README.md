# Exercise 1: Simulate Kinematic Model of Unicycle
In this exercise, we will trying to simulate the Kinematic Model of Unicycle in a ROS node. 

## 1) Kinematic Model
The Kinematic Model of Unicycle can be derived from the null basis of Pfaffian Matrix $A(q)$:

$$ 
\dot{q}(t) = S(q) \ \nu(t) \rightarrow 
\begin{pmatrix} 
\dot{x} \\
\dot{y} \\
\dot{\theta} \end{pmatrix} = \begin{pmatrix} 
                                \cos(\theta) & 0 \\
                                \sin(\theta) & 0 \\
                                0 & 1 
                            \end{pmatrix}
                                        \begin{pmatrix} 
                                            v \\
                                            \omega
                                        \end{pmatrix}
$$

In order to implement the equation in our node, we have to discretize this differential equation. Using Forward Euler, we can write:
$$\dot{q} \approx \frac{q(k+1) - q(k)}{T_s}$$

where $T_s$ is the sampling period. Finally, we can discretize the Kinematic Model of Unicycle:

$$q(k + 1) = q(k) + T_s \ S(q) \ \nu(k)$$

Expanding the previous equation:

$$\begin{cases}
x_{k+1} = x_k + T_s \ \cos(\theta_k) v_k \\
y_{k+1} = y_k + T_s \ \sin(\theta_k) v_k \\
\theta_{k+1} = \theta_{k} + T_s \ \omega_k
\end{cases}$$

The theoretical part is over, let's start to code.

## 2) Organize the code
The noide that we want to write, has as input $\nu = [v \quad \omega]^T$ and output $q = [x \quad y \quad \theta]^T$. We have to choice the ROS msgs for this node. A choice can be:
- Input: [geometry_msgs/Twist](https://docs.ros.org/en/api/geometry_msgs/html/msg/Twist.html)
- Output: [geometry_msgs/Pose2D](https://docs.ros.org/en/api/geometry_msgs/html/msg/Pose2D.html)

The messages definition can be found on the previous links.

## 3) Create pkg 
Let's create the package. Type in the terminal:
```
~/catkin_ws/src/ROS-Unipa$ catkin_create_pkg kinematic_unicycle roscpp std_msgs geometry_msgs
```

## 4) Create and build model node
Switch in `src` folder inside the `kinematic_unicycle` pkg. Create a new file called `model.cpp` and write the node. You can find the code [here](kinematic_unicycle/src/model.cpp). After saving the `.cpp` node, open the CMake file. After clear from the automatic comments (`#i'm a comment`), add these lines to build our node.
```
# This will create executable of the node
add_executable(model src/model.cpp)
# This will link executable to the appropriate libraries
target_link_libraries(model ${catkin_LIBRARIES})
```
After this, type in the terminal:
```
~/catkin_ws$ catkin_make
```
## 5) Execute the node
Open a terminal and type:
```
roscore
```
This instruction start rosmaster. Open another terminal and type:
```
rosrun kinematic_unicycle model
```
To control the simulated unicycle, execute the `teleop_twist_keyboard` [node](http://wiki.ros.org/teleop_twist_keyboard). To download it:
```
sudo apt-get install ros-noetic-teleop-twist-keyboard
```
Then, type:
```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```
## 6) Creating Launch file
Open 3 terminals is annoying. To avoid it, we can write a `.launch` file, that allow to launch more nodes. Inside the `kinematic_unicycle` pkg, create a `launch` folder and inside its, create `keyboard_control.launch`. You can find the code [here](kinematic_unicycle/launch/keyboard_control.launch). Finally, to launch the entire project, type in the terminal:

`$ roslaunch kinematic_model keyboard_control.launch`

## 7) Homework
- [x] Create a more complex control node, implementing a classic controller like path following or trajectory tracking.
- [ ] Create a sensor node that subscribe the state space of model node and add a white noise.

## 8) Control Node
During the lesson of 21/12/2022, **Giuseppe Valdes, Giorgia Comparato e Fabio Ammirata**, developed this [control node](kinematic_unicycle/src/controller). This algorithm control the motion of our unicycle in a straight line, using this control law:

$$ 
\omega_k = -K y_k - y_k \bar{v} \sinc(\theta_k)
$$

When if we want to modify the control gain, we have to rebuild our C++ nodes and can be annoying. To avoid this waste of time, we can include the control gain in the ROS Parameter server. To do this, we can follow these steps:

- 1. Create a folder inside your package, called `config`. Type in your terminal:
```
~/catkin_ws/src/kinematic_unicycle$ mkdir config
```
- 2. Create a new `.yaml` file, as an example `params.yaml`. In this file, you can store all of your parameters. You can check the code [here](kinematic_unicycle/config/params.yaml).

- 3. To get your parameters in your node, you need to use this syntax. 
```
ros::NodeHandle node_obj;

node_obj.getParam("<path_of_variable>", <name_of variable>)
```

In this way, you don't need to rebuild your package after a change of value. This is can be useful during the tuning process of a controller.