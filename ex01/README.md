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
Switch in `src` folder inside the `kinematic_unicycle` pkg. Create a new file called `model.cpp` and write the node. You can find the code [here](kinematic_unicycle/src/model.cpp). 

Modify CMake

Creating Launch file

`$ git clone https://github.com/Elektron97/ROS-Unipa.git`.
