#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import threading
import time
import cv2
import numpy as np
import syslog

# Global stop flag
stop_robot = False

# Opens syslog connection. Identifier, Add Log Pid, Only visible to current user
syslog.openlog(ident="MailDeliveryRobot", logoption=syslog.LOG_PID, facility=syslog.LOG_USER) 

# Initialize HOG descriptor for person detection
hog = cv2.HOGDescriptor()
hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

def log_robot(message):
    log_path = "/home/ubuntu/group2/myagv_ros/src/htmlFolder/log.txt"
    try:
        with open(log_path,"w", encoding="utf-8") as file:
            file.write(message)
    except Exception as e:
        syslog.syslog(syslog.LOG_ERR, "Failed to log messages: {e}")

def system_logging_info(message):
    syslog.syslog(syslog.LOG_INFO, message)

    

def person_detector():
    """
    Function to detect persons using HOG and publish stop signal.
    """
    global stop_robot

    # Initialize ROS node for detection
    stop_pub = rospy.Publisher('/stop_signal', Bool, queue_size=10)
    
    # Open webcam video stream
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        syslog.syslog(syslog.LOG_CRIT, "Critical: Camera initialization failed!")

    # Set video output parameters (if needed)
    out = cv2.VideoWriter(
        'output.avi',
        cv2.VideoWriter_fourcc(*'MJPG'),
        15.,
        (640,480)
    )

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            continue
        
        # Resize frame for faster detection
        frame = cv2.resize(frame, (640, 480))

        # Detect people in the image
        boxes, _ = hog.detectMultiScale(frame, winStride=(8,8))

        # Convert boxes to the format (xA, yA, xB, yB)
        boxes = np.array([[x, y, x + w, y + h] for (x, y, w, h) in boxes])

        # If any person is detected, send stop signal
        if len(boxes) > 0:
            stop_pub.publish(True)
        else:
            stop_pub.publish(False)

        # Draw the bounding boxes on the frame
        for (xA, yA, xB, yB) in boxes:
            cv2.rectangle(frame, (xA, yA), (xB, yB), (0, 255, 0), 2)
        
        # Optional: Write the frame to output file
        out.write(frame.astype('uint8'))

        # Display the frame
        cv2.imshow('Person Detection', frame)

        # Stop the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release resources when done
    cap.release()
    out.release()
    cv2.destroyAllWindows()

def stop_callback(msg):
    """
    Callback to handle stop signal updates.
    """
    global stop_robot
    stop_robot = msg.data

def move_agv(pub):
    """
    AGV movement control script with stop signal integration.
    """
    global stop_robot
    
    rospy.Subscriber('/stop_signal', Bool, stop_callback)
    rate = rospy.Rate(10)  # 10 Hz
    move_cmd = Twist()

    # Movement logic
    distance = 10  # Forward distance
    speed = rospy.get_param('~path_option',0.4)
    if not  (0.4 < speed <= 1.0): # Ensure no crazy speed input can happen.
        speed = 0.4
    duration = distance / speed
    move_cmd.linear.x = speed

    log_robot("Initiating robot...")
    system_logging_info("MailDeliveryRobot started successfully.")

    start_time = rospy.Time.now().to_sec()
    while rospy.Time.now().to_sec() - start_time < duration:
        if stop_robot:  # Check if the stop signal is active
            log_robot("Stopping AGV due to detected person.")
            # Stop the robot
            move_cmd.linear.x = 0.0
            pub.publish(move_cmd)
            
            duration = duration + 5
            # Wait for 5 seconds
            rospy.sleep(5)
            

            # Recheck stop signal
            if not stop_robot:
                log_robot("Resuming AGV movement.")
                system_logging_info("Resuming AGV movement.")
                move_cmd.linear.x = speed
            else:
                log_robot("Person still detected. Waiting.")
                continue

        pub.publish(move_cmd)
        rate.sleep()

    # Stop the robot after reaching the destination
    move_cmd.linear.x = 0.0
    pub.publish(move_cmd)
    rospy.sleep(1)
    log_robot("AGV movement completed.")
    system_logging_info("AGV movement completed.")
    

if __name__ == '__main__':
    try:
        # Initialize ROS node
        rospy.init_node('agv_control', anonymous=True)
        
        # Publishers
        stop_pub = rospy.Publisher('/stop_signal', Bool, queue_size=10)
        cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Run person detector in a separate thread
        detection_thread = threading.Thread(target=person_detector)
        detection_thread.daemon = True
        detection_thread.start()

        # Run the teleop control
        move_agv(cmd_vel_pub)
        log_robot(" ")
        syslog.syslog(syslog.LOG_INFO, "MailDeliveryRobot finished running successfully.")
        syslog.closelog()
    except rospy.ROSInterruptException:
        syslog.syslog(syslog.LOG_ERR, "Starting up ROS failed. Program exit.")
        syslog.closelog()
        pass