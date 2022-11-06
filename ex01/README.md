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

Finally, we can discretize the Kinematic Model of Unicycle:
$$q(k + 1) = q(k) + T_s \ S(q) \ \nu(k)$$

$$\begin{cases}
x_{k+1} = x_k + T_s \ \cos(\theta_k) v_k \\
y_{k+1} = y_k + T_s \ \sin(\theta_k) v_k \\
\theta_{k+1} = \theta_{k} + T_s \ \omega_k
\end{cases}$$

The theoretical part is over, let's start to code.

## 2) Organize the code

## 3) Create pkg

Create a pkg:
catkin_create_pkg kinematic_unicycle roscpp std_msgs geometry_msgs

Write Node...

Modify CMake

Creating Launch file

`$ git clone https://github.com/Elektron97/ROS-Unipa.git`.
