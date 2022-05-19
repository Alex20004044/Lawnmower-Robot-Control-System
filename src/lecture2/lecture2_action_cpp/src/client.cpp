#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <lecture2_actions/MoveAction.h>

void cbDone(const actionlib::SimpleClientGoalState& state, const lecture2_actions::MoveResultConstPtr& result)
{
    ROS_INFO_STREAM("DONE. State: " << state.toString());
    ROS_INFO_STREAM("result.position: " << result->position);
    ROS_INFO_STREAM("result.time:     " << result->time);
    ros::shutdown();
}

void cbActive()
{
    ROS_INFO("The action is active now.");
}

void cbFeedback(const lecture2_actions::MoveFeedbackConstPtr& feedback)
{
    ROS_INFO("FEEDBACK");
    ROS_INFO_STREAM("feedback.position: " << feedback->position);
    ROS_INFO_STREAM("feedback.speed:    " << feedback->speed);
    ROS_INFO_STREAM("feedback.elapsed:  " << feedback->elapsed);
    ROS_INFO_STREAM("feedback.remains:  " << feedback->remains);
}

int main (int argc, char **argv)
{
    ros::init(argc, argv, "move_client", ros::init_options::AnonymousName);
    actionlib::SimpleActionClient<lecture2_actions::MoveAction> sac("move", true);

    ROS_INFO("Waiting for the action server...");
    sac.waitForServer();

    ROS_INFO("Sending goal...");
    lecture2_actions::MoveGoal goal;
    goal.position = argc > 1 ? strtod(argv[1], nullptr) : 100.0;
    goal.speed = argc > 2 ? strtod(argv[2], nullptr) : 10.0;
    sac.sendGoal(goal, &cbDone, &cbActive, &cbFeedback);

    ros::spin();

    return 0;
}

