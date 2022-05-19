#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "publisher");
    ros::NodeHandle n;
    ros::Rate rate(30);
    const double X = 0.1;
    const double Y = 0.2;
    const double Z = 0.3;
    const double R = -0.5;
    const double A = 1.0;
    const double K = 2.0;

    tf2_ros::StaticTransformBroadcaster stb;
    geometry_msgs::TransformStamped sts;
    sts.header.stamp = ros::Time::now();
    sts.header.frame_id = "base_link";
    sts.child_frame_id = "head";
    sts.transform.translation.x = X;
    sts.transform.translation.y = Y;
    sts.transform.translation.z = Z;
    tf2::Quaternion sq;
    sq.setRPY(R, 0, 0);
    sts.transform.rotation.x = sq.x();
    sts.transform.rotation.y = sq.y();
    sts.transform.rotation.z = sq.z();
    sts.transform.rotation.w = sq.w();
    stb.sendTransform(sts);

    tf2_ros::TransformBroadcaster tb;
    double alpha = 0.0;
    while (ros::ok())
    {
        geometry_msgs::TransformStamped ts;
        ts.header.stamp = ros::Time::now();
        ts.header.frame_id = "world";
        ts.child_frame_id = "base_link";
        ts.transform.translation.x = A * cos(alpha);
        ts.transform.translation.y = A * sin(alpha);
        ts.transform.translation.z = 0.0;
        tf2::Quaternion q;
        q.setRPY(0, 0, K * alpha);
        ts.transform.rotation.x = q.x();
        ts.transform.rotation.y = q.y();
        ts.transform.rotation.z = q.z();
        ts.transform.rotation.w = q.w();
        tb.sendTransform(ts);
        rate.sleep();
        alpha += 0.01 * M_PI;
    }
    return 0;
}

