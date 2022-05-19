#!/usr/bin/env python3
import rospy
import tf_conversions
import tf2_ros
import geometry_msgs.msg
import math

def publisher():
    rospy.init_node("publisher")
    rate = rospy.Rate(30)
    X = 0.1
    Y = 0.2
    Z = 0.3
    R = -0.5
    A = 1.0
    K = 2.0

    stb = tf2_ros.StaticTransformBroadcaster()
    sts = geometry_msgs.msg.TransformStamped()
    sts.header.stamp = rospy.Time.now()
    sts.header.frame_id = "base_link"
    sts.child_frame_id = "head"
    sts.transform.translation.x = X
    sts.transform.translation.y = Y
    sts.transform.translation.z = Z
    sq = tf_conversions.transformations.quaternion_from_euler(R, 0, 0)
    sts.transform.rotation.x = sq[0]
    sts.transform.rotation.y = sq[1]
    sts.transform.rotation.z = sq[2]
    sts.transform.rotation.w = sq[3]
    stb.sendTransform(sts)

    tb = tf2_ros.TransformBroadcaster()
    alpha = 0.0
    while not rospy.is_shutdown():
        ts = geometry_msgs.msg.TransformStamped()
        ts.header.stamp = rospy.Time.now()
        ts.header.frame_id = "world"
        ts.child_frame_id = "base_link"
        ts.transform.translation.x = A * math.cos(alpha)
        ts.transform.translation.y = A * math.sin(alpha)
        ts.transform.translation.z = 0.0
        q = tf_conversions.transformations.quaternion_from_euler(0, 0, K * alpha)
        ts.transform.rotation.x = q[0]
        ts.transform.rotation.y = q[1]
        ts.transform.rotation.z = q[2]
        ts.transform.rotation.w = q[3]
        tb.sendTransform(ts)
        rate.sleep()
        alpha += 0.01 * math.pi;

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass

