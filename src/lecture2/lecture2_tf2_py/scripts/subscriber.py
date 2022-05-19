#!/usr/bin/env python3
import rospy
import tf2_ros
    
def subscriber():
    rospy.init_node("subscriber", anonymous=True)
    buf = tf2_ros.Buffer()
    tl = tf2_ros.TransformListener(buf)
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        try:
            ts = buf.lookup_transform("world", "head", rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(str(e))
            rospy.sleep(rospy.Duration(1))
            continue
        rospy.loginfo("%f\t%f\t%f", ts.transform.translation.x, ts.transform.translation.y, ts.transform.translation.z)
        rate.sleep()

if __name__ == '__main__':
    subscriber()

