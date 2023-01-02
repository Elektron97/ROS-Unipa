# Simulation on Gazebo of a 2 DoF Planar Manipulator
In this exercise, we will implement a 2 DoF Planar Manipulator on Gazebo. Let's start with some theory.

## Kinematics
Thanks to DH convention, we can write the **Forward Kinematics** of this type of manipulator:

![Scheme of a 2 DoF Planar Manipulator](robot_2DoF/docs/pics/2DoF_DH.png)

$$
T_{2}^{0}(\theta_1, \theta_2) = 
\begin{bmatrix}
c_{12} & -s_{12} & 0 & a_1 c_1 + a_2 c_{12} \\
s_{12} & c_{12} & 0 & a_1 s_1 + a_2 s_{12} \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1
\end{bmatrix} = 
\begin{bmatrix}
R_z(\theta_1 + \theta_2) & r_2^{0} \\
0^T & 1
\end{bmatrix} \in SE(3)
$$

This homogeneous matrix of $SE(3)$ group, expresses the roto-traslation from a fixed frame $\{S_0\}$ to $\{S_2\}$. As you can see, the kinematic parameters of the kinematic chain are $a_1$, $a_2$. The joint variables are $\theta_1$ and $\theta_2$. We can also define the joint variable vector:
$$
q(t) = \begin{bmatrix} \theta_1(t) \\ \theta_2(t) \end{bmatrix}
$$

From $T_2^0(q)$, we can compute also the Jacobian Matrix $J(q)$, that map the joint velocities in the twist of end-effector:

$$
\xi = \begin{bmatrix} v_x \\ v_y \\ \omega_z \end{bmatrix} = J(q) \dot{q}
$$

$$
J(q) = 
\begin{bmatrix}
-a_1 s_1 - a_2 s_{12} & -a_2 s_{12} \\
a_1 c_1 + a_2 c_{12} & a_2 c_{12} \\
1 & 1
\end{bmatrix}
$$

We can add also the joint limit in our model. So we can add the constraints:
$$
\bar{q}_{low} \leq q(t) \leq \bar{q}_{up}
$$

## Dynamics
Using the Lagrangian approach, we can write the Equation of Motion (EoM) of our robot:

$$
M(q) \ddot{q} + C(q, \dot{q}) \dot{q} + G(q) = \tau
$$
with boundary conditions: $q(0) = q_0 \quad \dot{q}(0) = \dot{q}_0$.

The expressions of these matrices, can be found [here](https://www.ijeert.org/papers/v6-i11/3.pdf) or in whatever robotics book. The parameters of dynamics are:
- $m_1$, $m_2$: masses of the links.
- $I_1$, $I_2$: Inertia tensor of the links. With simple geometry, like cylinder, we can express them with a simple expressions of the previous parameters.
- $p_{CoG, i}$: position of the center of gravity (CoG) of the $i$-th link.
- $g$: $9.81 \ m/s^2$.

*Note that in a realistic simulation, we have to add some other terms in the EoM, like elastic or damping terms ($K q + D \dot{q}$) or external forces ($J^Tf$).*

## Discretization
We can write our EoM in state-space form, defining:
$$  x_1 = q  \\
    x_2 = \dot{q} \\
    u = \tau
$$

From that:
$$
\begin{cases}
    \dot{x_1} = x_2 \\ 
    \dot{x_2} = - M^{-1}(x_1) \left( C(x_1, x_2) \ x_2  + G(x_1)\right) + M^{-1}(x_1) u
\end{cases}
$$

In compact form:
$$
\dot{x} = f(x) + g(x) u
$$

Where inertia matrix $M(q) \in R^{2 \times 2}$ is *positive definitive*. We can discretize with different discretization techniques. In Simulink, we choose an example *ode4* or other types of solver. The simplest is *Forward Euler* (in Simulink *ode1*, in the fixed step category). We can rewrite our model in algorithmic form:

$$
x_{k + 1} = x_k + T_s \left( f(x_k) + g(x_k) u_k \right)
$$

As you know, you have to choose a proper sampling period $T_s$ to avoid instability. The sampling period is another parameter of our simulation.

## Creation of the ROS pkg
After some theory, let's start to code. First, we create our package. So, type in your terminal:

```
cd catkin_ws/src
```
```
catkin_create_pkg robot_2DoF roscpp gazebo_msgs gazebo_plugins gazebo_ros gazebo_ros_control geometry_msgs sensor_msgs
```
There are a lot of dependencies. Some of these are gazebo-related. 

- `gazebo_ros`: This contains wrappers and tools for interfacing ROS with Gazebo.
- `gazebo_msgs`: Contains the definition of the messages of Gazebo, in order to interface with Gazebo from ROS.
- `gazebo_plugins`: Contains the dependencies needed to add sensors in our URDF model.
- `gazebo_ros_control`: This contains standard controllers to communicate between ROS and Gazebo.

There is also a new type of messages: `sensor_msgs`. You can find [here](http://wiki.ros.org/sensor_msgs) every type of standard sensor messages.

## Define Parameters
Open your package and create the `config` folder. Create `2dof_params.yaml`. Here we can add all parameters that we need. We can start to write only the kinematic and dynamic parameters. You can find the code [here](robot_2DoF/config/2dof_params.yaml).

## Build the Robot
Now we can build our robot. Create the `models` directory and create `params_macro.xacro`. In this file, we import the parameters from the `yaml` file and define our macro, in order to write clean and **readble code**.
