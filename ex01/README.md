# Exercise 1: Simulate Kinematic Model of Unicycle
In this exercise, we will trying to simulate the Kinematic Model of Unicycle in a ROS node. 

## 1) Kinematic Model
The Kinematic Model of Unicycle can be derived from the null basis of Pfaffian Matrix $$A(q)$$:

$$ \dot{q}(t) = S(q) \nu(t) \rightarrow \begin{bmatrix} \dot{x} \\ \dot{y} \\ \dot{\theta} \end{bmatrix} $$

Create a pkg:
catkin_create_pkg kinematic_unicycle roscpp std_msgs geometry_msgs

Write Node...

Modify CMake

Creating Launch file

`$ git clone https://github.com/Elektron97/ROS-Unipa.git`.
