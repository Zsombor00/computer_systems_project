#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import time

def move_agv():
    # Initialize the ROS node
    rospy.init_node('move_agv', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)  # Replace '/cmd_vel' with the correct topic for your AGV
    
    # Set rate to 10 Hz
    rate = rospy.Rate(10)
    
    # Define Twist message
    move_cmd = Twist()

    # Move forward 2 meters
    distance = 2.0  # Distance to move in meters
    speed = 0.2  # Linear speed in m/s
    duration = distance / speed  # Time to move forward
    move_cmd.linear.x = speed
    move_cmd.angular.z = 0.0

    start_time = rospy.Time.now().to_sec()
    while rospy.Time.now().to_sec() - start_time < duration:
        pub.publish(move_cmd)
        rate.sleep()

    # Stop the robot
    move_cmd.linear.x = 0.0
    pub.publish(move_cmd)
    rospy.sleep(1)

    # Move right 2 meters (sideways motion if your AGV supports holonomic movement)
    distance = 2.0  # Distance to move in meters
    speed = 0.2  # Linear speed in m/s for y-axis
    duration = distance / speed  # Time to move sideways
    move_cmd.linear.x = 0.0
    move_cmd.linear.y = speed  # Sideways motion (y-axis)

    start_time = rospy.Time.now().to_sec()
    while rospy.Time.now().to_sec() - start_time < duration:
        pub.publish(move_cmd)
        rate.sleep()

    # Stop the robot
    move_cmd.linear.y = 0.0
    pub.publish(move_cmd)
    rospy.sleep(1)

    rospy.loginfo("AGV movement completed.")

if __name__ == '__main__':
    try:
        move_agv()
    except rospy.ROSInterruptException:
        pass
