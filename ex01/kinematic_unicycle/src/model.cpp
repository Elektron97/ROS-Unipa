/**********MODEL OF UNICYCLE*********
 * This node simulate the kinematic *
 * model of unicycle.               *
 ************************************/

/*-------- INCLUDE --------*/
// ROS basics library
#include "ros/ros.h"

// ROS msgs
#include "geometry_msgs/Pose2D.h" 
#include "geometry_msgs/Twist.h"

/*-------- DEFINE --------*/
// ROS Node parameters
#define QUEUE_SIZE 10
#define FREQUENCY 1000.0 //float type

// Init Diff. Equation
#define X0 0.0
#define Y0 0.0
#define THETA0 0.0

#define Ts (1/FREQUENCY)

/*-------- GLOBAL VARIABLES --------*/
// State Space
geometry_msgs::Pose2D state_space;

// Control Input
float v;
float w;

/*-------- FUNCTION SIGNATURES --------*/
geometry_msgs::Pose2D kinematic_model(geometry_msgs::Pose2D q_prev, float vel, float omega);

/*-------- CALLBACKS --------*/
void vel_callBack(const geometry_msgs::Twist::ConstPtr& msg)
{
    // Update Control Input
    v = msg->linear.x;
    w = msg->angular.z;
    
    //ROS_INFO("Control Input received.");
    //ROS_INFO("v: %f, w: %f", v, w);
}

/*-------- MAIN --------*/
int main(int argc, char **argv)
{
    // --- Init node --- //
    ros::init(argc, argv, "model");
    ros::NodeHandle node_obj;

    // --- Communication --- //
    //Pub Objects
    ros::Publisher pub_state = node_obj.advertise<geometry_msgs::Pose2D>("/state_space", QUEUE_SIZE);

    //Sub Objects
    ros::Subscriber sub_ni = node_obj.subscribe("/cmd_vel", QUEUE_SIZE, vel_callBack);

    // Output:
    ROS_INFO("**************************");
    ROS_INFO("Model Node activated.");
    ROS_INFO("Init Pose of Unicycle:");
    ROS_INFO("[x0: %f, y0: %f, theta0: %f", X0, Y0, THETA0);
    ROS_INFO("**************************");

    // --- Loop Init --- //
    ros::Rate loop_rate(FREQUENCY);

    // Init State Space q(0)
    state_space.x       = X0;
    state_space.y       = Y0;
    state_space.theta   = THETA0;

    // --- Main Loop --- //
    // while(ros::ok()) is essentially while(1), until the node crash.
    while(ros::ok())
    {
        // Read Control Input
        ros::spinOnce();

        // Compute Kinematics
        // TC: q_dot(t) = A(q) * ni(t)
        // TD: (Euler1) q(k+1) = q(k) + Ts*A(q(k))*ni(k)
        state_space = kinematic_model(state_space, v, w);

        // Publish updated state space
        pub_state.publish(state_space);

        // Loop Rate
        loop_rate.sleep();
    }

    return 0;
}

geometry_msgs::Pose2D kinematic_model(geometry_msgs::Pose2D q_prev, float vel, float omega)
{
    /************************************************  
     *  This function is voluntarily not optimized  *
     *  for educational purpose.                    *
     ************************************************/
    
    // Define q(k+1)
    geometry_msgs::Pose2D q_upd;

    // Compute Kinematics
    q_upd.x = q_prev.x + Ts*cos(q_prev.theta)*vel;
    q_upd.y = q_prev.y + Ts*sin(q_prev.theta)*vel;
    q_upd.theta = q_prev.theta + Ts*omega;

    return q_upd;
}