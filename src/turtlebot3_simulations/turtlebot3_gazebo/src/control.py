#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
from auto.processor import ImageProcessor
from auto.tracker import Tracker
from auto.pid import PID
import numpy as np
import cv2
import time

# Image dimensions
IMAGE_WIDTH = 320
IMAGE_HEIGHT = 240

# Adjust the polynomial coefficients for the average positions based on the new image dimensions
averageLeft = np.poly1d(np.array([-1.725, 389.18]))  # Coefficients adjusted for new width
averageRight = np.poly1d(np.array([1.83, -164.07]))  # Coefficients adjusted for new width

# Initialize the processor, trackers, and PID controller
processor = ImageProcessor((IMAGE_WIDTH, IMAGE_HEIGHT), 30)
left_tracker = Tracker()
right_tracker = Tracker()
pid = PID(Kp=0.1, Ki=0.001, Kd=0.01)  # PID parameters adjusted
bridge = CvBridge()
last_time = time.time()

def callback(data):
    global last_time
    try:
        # Convert the ROS Image message to OpenCV image
        frame = bridge.imgmsg_to_cv2(data, 'bgr8')

        # Print the shape of the image
        print(f"Image shape: {frame.shape}")

        # Process the frame using the ImageProcessor class
        processor.process(frame)

        # Debug: Visualize lane detection
        if processor.left.poly is not None:
            processor.drawPoly(frame, processor.left.poly, (0, 255, 255))  # Draw detected left lane in yellow
        if processor.right.poly is not None:
            processor.drawPoly(frame, processor.right.poly, (255, 255, 0))  # Draw detected right lane in cyan

        # Check if lanes are detected
        if processor.left.poly is None or processor.right.poly is None:
            print("Lanes not detected properly.")
            return

        # Pass the found lanes to the Kalman Filter
        left_poly = left_tracker.add(processor.left.poly)
        right_poly = right_tracker.add(processor.right.poly)

        # Evaluate the polynomial at a certain height
        y_eval = int(processor.h * 0.9)  # Evaluate at 90% of the height
        left_eval = left_poly(y_eval)
        right_eval = right_poly(y_eval)
        center_x = int((left_eval + right_eval) / 2)
        image_center = frame.shape[1] / 2
        error = center_x - image_center

        # Print debug information
        print(f"Left eval: {left_eval}, Right eval: {right_eval}, Center X: {center_x}, Image Center: {image_center}, Error: {error}")

        # Draw the lanes and the center line
        processor.drawPoly(frame, left_poly, (0, 50, 255))  # Left lane in blue
        processor.drawPoly(frame, right_poly, (255, 50, 0))  # Right lane in red
        processor.drawPoly(frame, averageLeft, (255, 255, 255))  # Average left lane in white
        processor.drawPoly(frame, averageRight, (255, 255, 255))  # Average right lane in white
        cv2.line(frame, (center_x, frame.shape[0]), (center_x, int(frame.shape[0] * 0.6)), (0, 255, 0), 2)  # Center line in green

        # Compute the control signal using PID
        current_time = time.time()
        dt = current_time - last_time
        last_time = current_time

        control_signal = pid.compute(error, dt)
        print(f"Control Signal: {control_signal}")

        # Publish the movement command
        move = Twist()
        move.linear.x = 0.5  # Increase base linear speed
        move.angular.z = -control_signal
        pub.publish(move)

        # Display the processed frame
        cv2.imshow('frame', frame)
        cv2.waitKey(1)

    except CvBridgeError as e:
        print(e)

def writeCarStatus(frame, status):
    """
    Takes the status received from the car class and displays it on a frame.
    """
    if status:
        cv2.putText(frame, 'RPi: %.2f v' % (status['rpiBatteryVoltage']), (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 1)
        cv2.putText(frame, 'Motor: %.2f v' % (status['motorBatteryVoltage']), (10, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 1)
    return frame

if __name__ == '__main__':
    rospy.init_node('lane_follower')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rospy.Subscriber('/camera/image', Image, callback)

    rospy.spin()
    cv2.destroyAllWindows()
