#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

def draw_square():
    rospy.init_node('square_turtle', anonymous=True)
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    vel = Twist()

    while not rospy.is_shutdown():
        for i in range(4):
            # ✅ Turn first
            vel.linear.x = 0.0
            vel.angular.z = 1.57
            pub.publish(vel)
            rospy.sleep(1)

            # Stop rotation
            vel.angular.z = 0.0
            pub.publish(vel)
            rospy.sleep(0.5)

            # ➡️ Then move forward
            vel.linear.x = 2.0
            pub.publish(vel)
            rospy.sleep(3)

            # Stop movement
            vel.linear.x = 0.0
            pub.publish(vel)
            rospy.sleep(0.5)

if __name__ == '__main__':
    try:
        draw_square()
    except rospy.ROSInterruptException:
        pass
