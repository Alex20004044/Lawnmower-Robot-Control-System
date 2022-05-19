#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <lecture2_dr_cpp/DRConfig.h>

void callback(lecture2_dr_cpp::DRConfig &config, uint32_t level)
{
    ROS_INFO_STREAM(config.int_param << " " << config.double_param << " " << config.str_param << " " << (config.bool_param ? "True" : "False") << " " << config.enum_param);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "dr_node");
    dynamic_reconfigure::Server<lecture2_dr_cpp::DRConfig> server;
    server.setCallback(boost::bind(&callback, _1, _2));
    ros::spin();
    return 0;
}

