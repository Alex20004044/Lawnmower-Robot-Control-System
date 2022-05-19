#!/usr/bin/env python3
import rospy
import actionlib
import lecture2_actions.msg

class MoveAction:
    RATE = 10.0
    position = 0.0

    def __init__(self, name):
        self.sas = actionlib.SimpleActionServer(name, lecture2_actions.msg.MoveAction, execute_cb=self.callback, auto_start=False)
        self.sas.start()

    def callback(self, goal):
        speed = abs(goal.speed)
        dt = 1.0 / self.RATE
        dp = speed / self.RATE
        dp = dp if goal.position > self.position else -dp
        n = int(float(goal.position - self.position) / dp) 
        rate = rospy.Rate(self.RATE)
        for i in range(n):
            if self.sas.is_preempt_requested() or rospy.is_shutdown():
                self.sas.set_preempted()
                return
            feedback = lecture2_actions.msg.MoveFeedback()
            feedback.position = self.position
            feedback.speed = speed
            feedback.elapsed = i * dt
            feedback.remains = (n - i) * dt
            self.sas.publish_feedback(feedback)
            rate.sleep()
            self.position += dp
        result = lecture2_actions.msg.MoveResult()
        result.position = self.position
        result.time = n * dt
        self.sas.set_succeeded(result)

if __name__ == '__main__':
    rospy.init_node('move_server')
    move = MoveAction('move')
    rospy.spin()

