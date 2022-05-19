#include <ros/ros.h>
#include <std_srvs/SetBool.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "client", ros::init_options::AnonymousName);
    ros::NodeHandle n;
    ros::service::waitForService("service", -1);
    ros::ServiceClient c = n.serviceClient<std_srvs::SetBool>("service");
    std_srvs::SetBool srv;
    for (int i = 0; i <= 1; ++i)
    {
        srv.request.data = i;
        ROS_INFO_STREAM("Calling service: bool data = " << (i ? "True" : "False") << "...");
        if (c.call(srv))
        {
            ROS_INFO_STREAM("bool success: " << (srv.response.success ? "True" : "False"));
            ROS_INFO_STREAM("string message: " << srv.response.message);
        }
        else
        {
            ROS_ERROR("Service call failed");
        }
    }
    return 0;
}

