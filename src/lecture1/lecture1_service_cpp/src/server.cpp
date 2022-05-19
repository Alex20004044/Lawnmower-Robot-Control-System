#include <ros/ros.h>
#include <std_srvs/SetBool.h>

bool callback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp)
{
    resp.success = req.data;
    resp.message = "???";
    return true;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "server");
    ros::NodeHandle n;
    ros::ServiceServer s = n.advertiseService("service", callback);
    ros::spin();
    return 0;
}

