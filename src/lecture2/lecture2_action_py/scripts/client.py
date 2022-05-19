#!/usr/bin/env python3
import sys
import rospy
import actionlib
import lecture2_actions.msg

def cbDone(state, result):
    rospy.loginfo("DONE. State: %s", ["PENDING", "ACTIVE", "PREEMPTED", "SUCCEEDED", "ABORTED", "REJECTED", "PREEMPTING", "RECALLING", "RECALLED", "LOST"][state])
    rospy.loginfo("%s", str(result).replace('\n', '; '))
    rospy.signal_shutdown("quit");

def cbActive():
    rospy.loginfo("The action is active now.")

def cbFeedback(feedback):
    rospy.loginfo("FEEDBACK")
    rospy.loginfo("%s", str(feedback).replace('\n', '; '))

def client(position, speed):
    rospy.init_node('move_client', anonymous=True)
    sac = actionlib.SimpleActionClient('move', lecture2_actions.msg.MoveAction)
    rospy.loginfo("Waiting for the action server...")
    sac.wait_for_server()
    goal = lecture2_actions.msg.MoveGoal(position, speed)
    rospy.loginfo("Sending goal...")
    sac.send_goal(goal, cbDone, cbActive, cbFeedback)
    rospy.spin()

if __name__ == "__main__":
    position = float(sys.argv[1]) if len(sys.argv) > 1 else 100.0
    speed = float(sys.argv[2]) if len(sys.argv) > 2 else 10.0
    client(position, speed)

