#include <ros/ros.h>
#include <lecture1_srvs/StrCat.h>

bool callback(lecture1_srvs::StrCat::Request &req, lecture1_srvs::StrCat::Response &resp)
{
    resp.result = req.first + req.second;
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

