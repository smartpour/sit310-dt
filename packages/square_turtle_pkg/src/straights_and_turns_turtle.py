#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

class TurtleController:
    def __init__(self):
        self.pose = None
        self.goal_distance = None
        self.goal_angle = None
        self.start_pose = None
        self.state = 'idle'

        rospy.Subscriber('/turtle1/pose', Pose, self.pose_callback)
        rospy.Subscriber('/goal_distance', Float64, self.distance_callback)
        rospy.Subscriber('/goal_angle', Float64, self.angle_callback)

        self.pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.control_loop)

    def pose_callback(self, msg):
        self.pose = msg

    def distance_callback(self, msg):
        if self.pose:
            self.goal_distance = msg.data
            self.start_pose = self.pose
            self.state = 'moving'

    def angle_callback(self, msg):
        if self.pose:
            self.goal_angle = math.radians(msg.data)
            self.start_pose = self.pose
            self.state = 'rotating'

    def control_loop(self, event):
        if not self.pose:
            return

        vel = Twist()

        if self.state == 'moving' and self.goal_distance is not None:
            dx = self.pose.x - self.start_pose.x
            dy = self.pose.y - self.start_pose.y
            dist = math.sqrt(dx**2 + dy**2)

            if dist < abs(self.goal_distance):
                vel.linear.x = 2.0 if self.goal_distance > 0 else -2.0
            else:
                self.goal_distance = None
                self.state = 'idle'

        elif self.state == 'rotating' and self.goal_angle is not None:
            # Handle wraparound properly
            angle_diff = self.normalize_angle(self.pose.theta - self.start_pose.theta)

            if abs(angle_diff) < abs(self.goal_angle):
                vel.angular.z = 1.57 if self.goal_angle > 0 else -1.57
            else:
                self.goal_angle = None
                self.state = 'idle'

        self.pub.publish(vel)

    def normalize_angle(self, angle):
        """Normalize angle to be between -pi and pi"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

if __name__ == '__main__':
    rospy.init_node('straights_and_turns_turtle')
    TurtleController()
    rospy.spin()
