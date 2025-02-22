#include <ros/ros.h>

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "hello_world");
    ros::NodeHandle nh;
    
    ROS_INFO("Hello World!");
    
    ros::spin();
    return 0;
} 