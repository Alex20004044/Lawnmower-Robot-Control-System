#!/usr/bin/env python3
import cv2
import numpy as np
import rospy
import tf2_ros
import actionlib
from tf_conversions import transformations
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
from geometry_msgs.msg import Point, Quaternion, Pose, PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class Autopilot:

    def __init__(self):
        self.sac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.sac.wait_for_server()
        self.tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf_buffer)
        rospy.Subscriber("map", OccupancyGrid, self.callback)

    def callback(self, og):
        rospy.loginfo("Map update received...")
        robot_position = None
        while robot_position is None:
            try:
                robot_position = self.tf_buffer.lookup_transform("map", "base_link", rospy.Time()).transform.translation
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                rospy.logwarn(str(e))
                rospy.sleep(1)
        rospy.loginfo("Robot coordinates: %f, %f", robot_position.x, robot_position.y)
        resolution = og.info.resolution
        position = og.info.origin.position
        orientation = og.info.origin.orientation
        img = np.array(og.data).reshape(og.info.height, og.info.width)
        img = (img < 50) * 255 - (img == -1) * (255 - 205)
        img = img.astype('uint8')
        _, img_wall = cv2.threshold(img, 127, 255, cv2.THRESH_BINARY)
        img_wall = cv2.erode(img_wall, np.ones((11, 11), np.uint8))
        _, img_empty = cv2.threshold(img, 220, 255, cv2.THRESH_BINARY)
        img_empty = cv2.morphologyEx(img_empty, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        img_empty = cv2.morphologyEx(img_empty, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
        img_edge = cv2.Canny(img_empty, 100, 200)
        img_target = np.logical_and(img_wall > 127, img_edge > 127) * 255
        img_target = img_target.astype('uint8')
        contours, _ = cv2.findContours(img_target, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) == 0:
            rospy.loginfo("Autopilot is shutting down...")
            self.sac.cancel_goal()
            rospy.signal_shutdown("quit")
            return
        boxes = [cv2.boundingRect(contour) for contour in contours]
        points = [(x + w / 2, y + h / 2) for x, y, w, h in boxes]
        translation = np.array([[1.0, 0.0, 0.0, position.x], [0.0, 1.0, 0.0, position.y], [0.0, 0.0, 1.0, position.z], [0.0, 0.0, 0.0, 1.0]])
        rotation = transformations.quaternion_matrix([orientation.x, orientation.y, orientation.z, orientation.w])
        transformation = np.dot(translation, rotation)
        positions = [np.dot(transformation, np.array([x * resolution, y * resolution, 0.0, 1.0])) for x, y in points]
        goals = [(x / w, y / w) for x, y, _, w in positions]
        ds = [(robot_position.x - x) ** 2 + (robot_position.y - y) ** 2 for x, y in goals]
        goal = goals[ds.index(min(ds))]
        rospy.loginfo("Sending new goal: %f, %f", goal[0], goal[1])
        goal_msg = MoveBaseGoal(PoseStamped(Header(0, rospy.Time(), "map"), Pose(Point(goal[0], goal[1], 0), Quaternion(0, 0, 0, 1))))
        self.sac.send_goal(goal_msg)

if __name__ == "__main__":
    rospy.init_node("autopilot")
    rospy.sleep(10)
    Autopilot()
    rospy.spin()

