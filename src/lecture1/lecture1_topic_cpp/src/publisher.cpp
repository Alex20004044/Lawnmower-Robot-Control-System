#include <ros/ros.h>
#include <std_msgs/Int32.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "publisher");
    ros::NodeHandle n;
    ros::Publisher p = n.advertise<std_msgs::Int32>("topic", 5);
    ros::Rate rate(10);
    int c = 0;
    while (ros::ok())
    {
        std_msgs::Int32 msg;
        msg.data = c++;
        p.publish(msg);
        rate.sleep();
    }
    return 0;
}

