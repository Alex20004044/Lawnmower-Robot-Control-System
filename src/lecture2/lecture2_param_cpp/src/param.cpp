#include <ros/ros.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "node");
    ros::NodeHandle nh("~");
    std::string s;
    nh.param<std::string>("param", s, "default_value");
    ROS_INFO_STREAM(s);
    return 0;
}

