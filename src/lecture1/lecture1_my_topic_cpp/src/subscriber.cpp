#include <ros/ros.h>
#include <lecture1_msgs/TimeUInt32.h>

void callback(const lecture1_msgs::TimeUInt32::ConstPtr& msg)
{
    ROS_INFO_STREAM(msg->stamp << " : " << msg->data);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "subscriber", ros::init_options::AnonymousName);
    ros::NodeHandle n;
    ros::Subscriber s = n.subscribe("topic", 5, callback);
    ros::spin();
    return 0;
}

