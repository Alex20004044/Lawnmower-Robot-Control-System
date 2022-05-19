#include <ros/ros.h>
#include <lecture1_msgs/TimeUInt32.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "publisher");
    ros::NodeHandle n;
    ros::Publisher p = n.advertise<lecture1_msgs::TimeUInt32>("topic", 5);
    ros::Rate rate(10);
    int c = 0;
    while (ros::ok())
    {
        lecture1_msgs::TimeUInt32 msg;
        msg.stamp = ros::Time::now();
        msg.data = c++;
        p.publish(msg);
        rate.sleep();
    }
    return 0;
}

