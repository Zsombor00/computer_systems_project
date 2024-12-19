#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import time
from std_msgs.msg import String
import sys
print("Python vers", sys.version)

# Initialize the ROS node
rospy.init_node('move_agv', anonymous=True)
log_pub_movement = rospy.Publisher('/movementlogs', String, queue_size=10)
log_pub_progress = rospy.Publisher('/progresslogs', String, queue_size=10)
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
# Set rate to 10 Hz
rate = rospy.Rate(10)

# Global variables
speed_multiplier = 0.5
selected_route = "A"

def log_and_publish_movement(msg):
    rospy.loginfo(msg)
    log_pub_movement.publish(msg)

def log_and_publish_progress(msg):
    rospy.loginfo(msg)
    log_pub_progress.publish(msg)


def rotate_to_face(direction):
    """
    Rotate the robot to face the desired direction.
    :param direction: The target direction ("forward", "right", "left", or "backward")
    """
    rotate_cmd = Twist()
    angular_speed = 0.5  # Adjust as needed
    duration = 0  # Duration to rotate (in seconds)

    if direction == "forward":
        duration = 0  # No rotation needed
    elif direction == "left":
        duration = 6  # Adjust this to achieve a 90-degree right turn
    elif direction == "right":
        duration = 6  # Adjust this to achieve a 90-degree left turn
        angular_speed = -0.5  # Rotate counter-clockwise
    elif direction == "backward":
        duration = 12  # Adjust this to achieve a 180-degree turn

    # Rotate the robot
    rotate_cmd.angular.z = angular_speed
    start_time = rospy.Time.now().to_sec()
    while rospy.Time.now().to_sec() - start_time < duration:
        pub.publish(rotate_cmd)
        rate.sleep()

    # Stop rotation
    rotate_cmd.angular.z = 0.0
    pub.publish(rotate_cmd)

def move_agv():    
    # Define Twist message
    move_cmd = Twist()

    # Define routes
    routes = {
        "A": [
            {"distance": 6.0, "linear_x": 0.4, "linear_y": 0.0, "direction": "forward", "description": "Moving forward"}  # Forward
        ]
    }

    # Get the selected route's movements
    movements = routes[selected_route]

    for move in movements:
        distance = move["distance"]
        speed_x = move["linear_x"]
        speed_y = move["linear_y"]
        direction = move["direction"]
        description = move["description"]

        # Rotate to face the direction of movement
        rotate_to_face(direction)

        # Calculate duration based on speed
        duration = distance / abs(speed_x * speed_multiplier if speed_x != 0 else speed_y * speed_multiplier)
        
        # Adjust speed with multiplier
        move_cmd.linear.x = speed_x * speed_multiplier
        move_cmd.linear.y = speed_y * speed_multiplier

        # log_and_publish_movement(f"Starting: {description} for {distance} meters at speed multiplier {speed_multiplier}")
        
        start_time = rospy.Time.now().to_sec()

        # Publish movement commands
        while rospy.Time.now().to_sec() - start_time < duration:
            pub.publish(move_cmd)
            rate.sleep()

        # Stop the robot
        move_cmd.linear.x = 0.0
        move_cmd.linear.y = 0.0
        pub.publish(move_cmd)

if __name__ == '__main__':
    try:
        move_agv()
    except rospy.ROSInterruptException:
        pass