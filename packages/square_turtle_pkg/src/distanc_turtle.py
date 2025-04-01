#!/usr/bin/env python3

import rospy
from turtlesim.msg import Pose
from std_msgs.msg import Float64
import math

class DistanceTracker:
    def __init__(self):
        self.last_pose = None
        self.total_distance = 0.0

        rospy.Subscriber('/turtle1/pose', Pose, self.pose_callback)
        self.distance_pub = rospy.Publisher('/turtle_distance', Float64, queue_size=10)

    def pose_callback(self, msg):
        if self.last_pose is not None:
            dx = msg.x - self.last_pose.x
            dy = msg.y - self.last_pose.y
            dist = math.sqrt(dx**2 + dy**2)
            self.total_distance += dist

        self.last_pose = msg
        self.distance_pub.publish(self.total_distance)

if __name__ == '__main__':
    rospy.init_node('distance_turtle', anonymous=True)
    DistanceTracker()
    rospy.spin()
