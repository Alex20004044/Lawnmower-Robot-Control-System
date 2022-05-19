#include <ros/ros.h>
#include <lecture1_srvs/StrCat.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "client", ros::init_options::AnonymousName);
    ros::NodeHandle n;
    ros::service::waitForService("service", -1);
    ros::ServiceClient c = n.serviceClient<lecture1_srvs::StrCat>("service");
    lecture1_srvs::StrCat srv;
    if (argc > 1)
    {
        srv.request.first = argv[1];
    }
    if (argc > 2)
    {
        srv.request.second = argv[2];
    }
    ROS_INFO_STREAM("Calling service for \"" << srv.request.first << "\" and \"" << srv.request.second << "\"...");
    if (c.call(srv))
    {
        ROS_INFO_STREAM("Result: \"" << srv.response.result << "\"");
    }
    else
    {
        ROS_ERROR("Service call failed");
    }
    return 0;
}

