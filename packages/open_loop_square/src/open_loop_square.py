#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import FSMState

class Drive_Square:
    def __init__(self):
        # Initialize global class variables
        self.cmd_msg = Twist2DStamped()

        # Initialize ROS node
        rospy.init_node('drive_square_node', anonymous=True)
        
        # Initialize Pub/Subs
        self.pub = rospy.Publisher('/akandb/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        rospy.Subscriber('/akandb/fsm_node/mode', FSMState, self.fsm_callback, queue_size=1)
        
    # Robot only moves when lane following is selected on the Duckiebot joystick app
    def fsm_callback(self, msg):
        rospy.loginfo("State: %s", msg.state)
        if msg.state == "NORMAL_JOYSTICK_CONTROL":
            self.stop_robot()
        elif msg.state == "LANE_FOLLOWING":
            rospy.sleep(1)  # Wait for a sec for the node to be ready
            self.move_robot()

    # Sends zero velocities to stop the robot
    def stop_robot(self):
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.0
        self.cmd_msg.omega = 0.0
        self.pub.publish(self.cmd_msg)

    # Robot drives in a square and then stops
    def move_robot(self):
        # Move forward 1 meter
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.5  # Straight line velocity
        self.cmd_msg.omega = 0.0
        self.pub.publish(self.cmd_msg)
        rospy.loginfo("Moving forward!")
        rospy.sleep(2)  # Adjust this sleep time based on your robot's speed to move 1 meter

        # Turn 90 degrees to the left
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.0
        self.cmd_msg.omega = 0.5  # Adjust omega for turning
        self.pub.publish(self.cmd_msg)
        rospy.loginfo("Turning left!")
        rospy.sleep(1)  # Adjust this time for a 90-degree turn

        # Move forward 1 meter
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.5
        self.cmd_msg.omega = 0.0
        self.pub.publish(self.cmd_msg)
        rospy.loginfo("Moving forward!")
        rospy.sleep(2)

        # Turn 90 degrees to the left
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.0
        self.cmd_msg.omega = 0.5
        self.pub.publish(self.cmd_msg)
        rospy.loginfo("Turning left!")
        rospy.sleep(1)

        # Move forward 1 meter
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.5
        self.cmd_msg.omega = 0.0
        self.pub.publish(self.cmd_msg)
        rospy.loginfo("Moving forward!")
        rospy.sleep(2)

        # Turn 90 degrees to the left
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.0
        self.cmd_msg.omega = 0.5
        self.pub.publish(self.cmd_msg)
        rospy.loginfo("Turning left!")
        rospy.sleep(1)

        # Stop the robot
        self.stop_robot()

    # Spin forever but listen to message callbacks
    def run(self):
        rospy.spin()  # Keeps the node from exiting until the node has shutdown

if __name__ == '__main__':
    try:
        duckiebot_movement = Drive_Square()
        duckiebot_movement.run()
    except rospy.ROSInterruptException:
        pass
