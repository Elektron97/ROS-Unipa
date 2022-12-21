/************************************************
 *                CONTROLLER NODE               *
 * This node was developed by Giuseppe Valdes,  *
 * Giorgia Comparato and Fabio Ammirata,        *
 * in 21/12/2022.                               *
 ************************************************/

// Include
#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h" 
#include "geometry_msgs/Twist.h"

// Parameters
#define FREQUENCY 1000.0
#define QUEUE_SIZE 10

// Tunable parameters
int VBAR;
int K;

// Global Variables
float yr;
float thetar;

geometry_msgs::Twist w;

// Function Signature
geometry_msgs::Twist control_law(float yy, float tt);
float sinc(float x);

// callBack
void state_callBack(const geometry_msgs::Pose2D::ConstPtr& msg)
{
    // Extract data
    yr = msg->y;
    thetar = msg->theta;
}

/*MAIN*/
int main(int argc, char **argv)
{

    ros::init(argc, argv, "controller");
    ros::NodeHandle node_obj;

    ros::Publisher pub_state = node_obj.advertise<geometry_msgs::Twist>("/cmd_vel", QUEUE_SIZE);

    ros::Subscriber sub_state = node_obj.subscribe("/state_space", QUEUE_SIZE, state_callBack);

    ROS_INFO("**************************");
    ROS_INFO("Controller Node activated.");
    ROS_INFO("**************************");

    ros::Rate loop_rate(FREQUENCY);

    /*LOAD PARAMETERS FROM ROSPARAMETERS SERVER*/
    node_obj.getParam("/vbar", VBAR);
    node_obj.getParam("/control_gain", K);

    // Main Loop
    while(ros::ok())
    {
        // Read State Space
        ros::spinOnce();

        // Compute Control Law
        w = control_law(yr, thetar);
        // Constant velocity
        w.linear.x = VBAR;

        // Pub velocity command
        pub_state.publish(w);

        loop_rate.sleep();
    }

    return 0;
}

float sinc(float x) {
    if (x == 0)
        return 1.0;
    else
        return sin(x)/x;
}

geometry_msgs::Twist control_law(float yy, float tt)
{
    geometry_msgs::Twist vel;

    // Compute Control Law
    // omega(k) = -K*theta(k) - y(k)*v*sinc(theta(k))
    vel.angular.z = -K*tt - yy*VBAR*sinc(tt);

    return vel;
}