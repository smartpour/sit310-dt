#!/usr/bin/env python
import time
from duckietown_msgs.msg import WheelEncoderStamped, Twist2DStamped
from std_msgs.msg import Header
import threading

# Global variables to store the encoder ticks
current_ticks_right = 0
current_ticks_left = 0

# Define the publisher for controlling the robot's movement
pub = None

# Callback function for encoder ticks (left wheel)
def encoder_callback_left(data):
    global current_ticks_left
    current_ticks_left = data.data  # Update the left wheel encoder ticks

# Callback function for encoder ticks (right wheel)
def encoder_callback_right(data):
    global current_ticks_right
    current_ticks_right = data.data  # Update the right wheel encoder ticks

# Function to move the robot a specific distance in a straight line
def move_straight(distance, speed):
    # Adjust these values based on your robot's actual configuration
    ticks_per_meter = 100  # (Example value, needs calibration)
    current_ticks = 0
    goal_ticks = distance * ticks_per_meter  # Calculate goal ticks for the desired distance

    # Create a Twist2DStamped message to move the robot forward
    msg = Twist2DStamped()
    msg.header = Header()
    msg.v = speed  # Set the forward speed
    msg.omega = 0  # No rotation

    # Publish the forward movement command
    pub.publish(msg)
    print(f"Moving straight for {distance} meters...")

    # Wait until the robot reaches the goal distance
    while abs(current_ticks) < goal_ticks:
        time.sleep(0.1)  # Sleep for a short time to receive encoder data
        print(f"Current ticks: {current_ticks}, Goal ticks: {goal_ticks}")

    # Stop the robot once the goal is reached
    msg.v = 0
    pub.publish(msg)
    print("Reached goal distance, stopping robot.")

# Function to rotate the robot in place by a specific angle
def rotate_in_place(angle, speed):
    # Adjust these values based on your robot's actual configuration
    ticks_per_degree = 10  # (Example value, needs calibration)
    current_ticks = 0
    goal_ticks = angle * ticks_per_degree  # Calculate goal ticks for the desired angle

    # Create a Twist2DStamped message to rotate the robot
    msg = Twist2DStamped()
    msg.header = Header()
    msg.v = 0  # No forward motion
    msg.omega = speed  # Set the angular speed

    # Publish the rotation command
    pub.publish(msg)
    print(f"Rotating for {angle} degrees...")

    # Wait until the robot completes the rotation
    while abs(current_ticks) < goal_ticks:
        time.sleep(0.1)  # Sleep for a short time to receive encoder data
        print(f"Current ticks: {current_ticks}, Goal ticks: {goal_ticks}")

    # Stop the robot after rotation
    msg.omega = 0
    pub.publish(msg)
    print("Reached goal angle, stopping robot.")

# Function to draw a square with 1-meter sides
def draw_square():
    print("Starting square drawing.")
    move_straight(1, 0.2)  # Move forward 1 meter at a speed of 0.2 m/s
    rotate_in_place(90, 0.5)  # Rotate 90 degrees at a speed of 0.5 rad/s
    move_straight(1, 0.2)  # Move forward 1 meter
    rotate_in_place(90, 0.5)  # Rotate 90 degrees
    move_straight(1, 0.2)  # Move forward 1 meter
    rotate_in_place(90, 0.5)  # Rotate 90 degrees
    move_straight(1, 0.2)  # Move forward 1 meter to complete the square
    print("Square drawing complete.")

def setup_pub_and_subscribers():
    # Initialize the publisher for robot commands
    global pub
    pub = rospy.Publisher('/robobae/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=10)

    # Initialize the subscribers for encoder topics (left and right wheels)
    rospy.Subscriber('/robobae/left_wheel_encoder_node/tick', WheelEncoderStamped, encoder_callback_left)
    rospy.Subscriber('/robobae/right_wheel_encoder_node/tick', WheelEncoderStamped, encoder_callback_right)

    # Wait for the first message from the encoders to ensure data is being received
    time.sleep(1)

def main():
    # Initialize ROS Node and start the publisher/subscriber
    setup_pub_and_subscribers()

    # Draw the square
    draw_square()

if __name__ == '__main__':
    main()
