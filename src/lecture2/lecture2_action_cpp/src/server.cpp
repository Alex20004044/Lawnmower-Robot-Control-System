#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <lecture2_actions/MoveAction.h>

class MoveAction
{
private:
    const double RATE = 10.0;
    ros::NodeHandle nh;
    actionlib::SimpleActionServer<lecture2_actions::MoveAction> sas;
    double position = 0;

public:
    MoveAction(std::string name) : sas(nh, name, boost::bind(&MoveAction::callback, this, _1), false)
    {
        sas.start();
    }
    void callback(const lecture2_actions::MoveGoalConstPtr &goal)
    {
        double speed = abs(goal->speed);
        double dt = 1.0 / RATE;
        double dp = speed / RATE;
        dp = (goal->position > position) ? dp : -dp;
        unsigned int n = (float) (goal->position - position) / dp; 
        ros::Rate rate(RATE);
        for (unsigned int i = 0; i < n; ++i)
        {
            if (sas.isPreemptRequested() || !ros::ok())
            {
                sas.setPreempted();
                return;
            }
            lecture2_actions::MoveFeedback feedback;
            feedback.position = position;
            feedback.speed = speed;
            feedback.elapsed = i * dt;
            feedback.remains = (n - i) * dt;            
            sas.publishFeedback(feedback);
            rate.sleep();
            position += dp;
        }
        lecture2_actions::MoveResult result;
        result.position = position;
        result.time = n * dt;
        sas.setSucceeded(result);
    }
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "move_server");
    MoveAction move("move");
    ros::spin();
    return 0;
}

