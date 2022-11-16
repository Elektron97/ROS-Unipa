/********PUBLISHER NODE********/

/*INCLUDE*/
//ros
#include "ros/ros.h"
//ROS Msgs used in this node
#include "std_msgs/String.h"

/*DEFINE*/
#define NODE_FREQUENCY 100 //[Hz]
#define QUEUE_SIZE 10

/*MAIN*/
int main(int argc, char **argv)
{
    // Init node
    ros::init(argc, argv, "talker");
    // Node Object
    ros::NodeHandle node_obj;

    // Define Pub Objects
    ros::Publisher string_pub = node_obj.advertise<std_msgs::String>("string_topic", QUEUE_SIZE);

    // Rate Object
    ros::Rate loop_rate(NODE_FREQUENCY);

    // Init Variables
    std_msgs::String msg;
    msg.data = "Hello World!";
    // Counter
    int count = 0;

    // Main Loop
    while(ros::ok())
    {        
        // Pub String msg in /string_topic
        string_pub.publish(msg);
        
        // spinOnce is necessary to subscribe topics. 
        // For publish-only node, we can comment this line of code.
        //ros::spinOnce();

        // sleep for the time remaining to let us hit our NODE_FREQUENCY publish rate
        loop_rate.sleep();

        // Count how much topic we have published
        ++count;

        // Screen Output
        ROS_INFO("%s | %d-th msg", msg.data.c_str(), count);
    }

    // End of Node
    return 0;
}