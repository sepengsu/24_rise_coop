#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import time

# Image dimensions
IMAGE_WIDTH = 320
IMAGE_HEIGHT = 240

# PID controller parameters
Kp = 0.01
Ki = 0.001
Kd = 0.01

bridge = CvBridge()
last_time = time.time()

def detect_lanes(image):
    # 이미지를 그레이스케일로 변환합니다.
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Canny 엣지 검출을 수행합니다.
    edges = cv2.Canny(gray, 50, 150)

    # Hough 변환을 사용하여 선을 검출합니다.
    lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=50, minLineLength=50, maxLineGap=20)

    left_lines = []
    right_lines = []

    # 검출된 선을 분류합니다.
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            slope = (y2 - y1) / (x2 - x1)
            if slope < 0:
                left_lines.append(line)
            else:
                right_lines.append(line)

    return left_lines, right_lines

def average_slope_intercept(lines):
    x_points = []
    y_points = []
    for line in lines:
        x1, y1, x2, y2 = line[0]
        x_points.extend([x1, x2])
        y_points.extend([y1, y2])
    
    if len(x_points) == 0 or len(y_points) == 0:
        return None

    poly = np.poly1d(np.polyfit(y_points, x_points, 1))
    return poly

def compute_control_signal(left_poly, right_poly, image_height):
    y_eval = int(image_height * 0.9)
    left_x = left_poly(y_eval) if left_poly is not None else 0
    right_x = right_poly(y_eval) if right_poly is not None else IMAGE_WIDTH
    center_x = (left_x + right_x) / 2
    image_center = IMAGE_WIDTH / 2
    error = center_x - image_center
    return error

def callback(data):
    global last_time
    try:
        # Convert the ROS Image message to OpenCV image
        frame = bridge.imgmsg_to_cv2(data, 'bgr8')

        # Print the shape of the image
        print(f"Image shape: {frame.shape}")

        # Detect lanes
        left_lines, right_lines = detect_lanes(frame)

        # Compute average slope and intercept for left and right lanes
        left_poly = average_slope_intercept(left_lines)
        right_poly = average_slope_intercept(right_lines)

        # Draw the lanes
        if left_poly is not None:
            y1 = IMAGE_HEIGHT
            y2 = int(IMAGE_HEIGHT * 0.6)
            x1 = int(left_poly(y1))
            x2 = int(left_poly(y2))
            cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 255), 2)

        if right_poly is not None:
            y1 = IMAGE_HEIGHT
            y2 = int(IMAGE_HEIGHT * 0.6)
            x1 = int(right_poly(y1))
            x2 = int(right_poly(y2))
            cv2.line(frame, (x1, y1), (x2, y2), (255, 255, 0), 2)

        # Compute control signal using PID
        error = compute_control_signal(left_poly, right_poly, IMAGE_HEIGHT)
        print(f"Control Error: {error}")

        # PID Control
        current_time = time.time()
        dt = current_time - last_time
        last_time = current_time

        integral = 0
        integral += error * dt
        derivative = error / dt

        control_signal = Kp * error + Ki * integral + Kd * derivative
        print(f"Control Signal: {control_signal}")

        # Publish the movement command
        move = Twist()
        move.linear.x = 0.1  # 기본 선형 속도
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
