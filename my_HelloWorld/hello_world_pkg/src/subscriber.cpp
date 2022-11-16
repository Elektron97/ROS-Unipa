/********PUBLISHER NODE********/

/*INCLUDE*/
//ros
#include "ros/ros.h"
//ROS Msgs used in this node
#include "std_msgs/String.h"

/*DEFINE*/
#define QUEUE_SIZE 10

/*CALLBACK*/
void string_callBack(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("I heard: [%s]", msg->data.c_str());
}

/*MAIN*/
int main(int argc, char **argv)
{
    // Init node
    ros::init(argc, argv, "subscriber");
    // Node Object
    ros::NodeHandle node_obj;

    // Define Sub Objects
    ros::Subscriber string_sub = node_obj.subscribe("string_topic", QUEUE_SIZE, string_callBack);

    // ros::spin() will enter a loop,
    // waiting and subscribing the topics.
    ros::spin();

    // End of Node
    return 0;
}