#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "subscriber", ros::init_options::AnonymousName);
    ros::NodeHandle n;
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener tl(buffer);
    ros::Rate rate(30);
    while (ros::ok())
    {
        geometry_msgs::TransformStamped ts;
        try
        {
            ts = buffer.lookupTransform("world", "head", ros::Time(0));
        }
        catch (tf2::TransformException &e)
        {
            ROS_WARN("%s", e.what());
            ros::Duration(1).sleep();
            continue;
        }
        ROS_INFO_STREAM(ts.transform.translation.x << "\t" << ts.transform.translation.y << "\t" << ts.transform.translation.z);
        rate.sleep();
    }
    return 0;
}
