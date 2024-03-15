#!/usr/bin/python3

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from std_msgs.msg import Float32

class LaneDetectorNode:
    def __init__(self):
        rospy.init_node('lane_detector_node')

        self.image_sub = rospy.Subscriber('/m2d/camera_top/image', Image, self.image_callback)
        self.image_pub = rospy.Publisher('/lane_detection/image', Image, queue_size=1)
        self.steering_pub = rospy.Publisher('/lane_detection/steering_angle', Float32, queue_size=1)

        self.bridge = CvBridge()

        # ROI parameters
        self.roi_top = rospy.get_param('~roi_top', 0.6)
        self.roi_bottom = rospy.get_param('~roi_bottom', 1.0)

        # Canny edge detection parameters
        self.canny_threshold = rospy.get_param('~canny_threshold', 50)

        # Hough transform parameters
        self.hough_threshold = rospy.get_param('~hough_threshold', 50)
        self.min_line_length = rospy.get_param('~min_line_length', 50)
        self.max_line_gap = rospy.get_param('~max_line_gap', 30)

        # Steering control parameters
        self.steering_gain = rospy.get_param('~steering_gain', 0.1)
        self.max_steering_angle = rospy.get_param('~max_steering_angle', np.pi/6)

    def image_callback(self, msg):
        # Convert the ROS image message to OpenCV image
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Convert the image to grayscale
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Crop the region of interest (ROI)
        height, width = gray.shape
        roi_height = int(height * (self.roi_bottom - self.roi_top))
        roi_y_offset = int(height * self.roi_top)
        roi = gray[roi_y_offset:(roi_y_offset+roi_height), :]

        # Apply Gaussian blur to reduce noise
        blur = cv2.GaussianBlur(roi, (3, 3), 0)

        # Apply Canny edge detection to find edges
        edges = cv2.Canny(blur, self.canny_threshold, self.canny_threshold*3)

        # Apply Hough transform to find lane lines
        lines = cv2.HoughLinesP(edges, 1, np.pi/180, self.hough_threshold, self.min_line_length, self.max_line_gap)

        # Compute the slope and intercept of each line
        left_slope_sum = 0
        left_intercept_sum = 0
        left_count = 0
        right_slope_sum = 0
        right_intercept_sum = 0
        right_count = 0
        for line in lines:
            x1, y1, x2, y2 = line[0]
            slope = (y2 - y1) / (x2 - x1)
            intercept = y1 - slope * x1
            if slope < -0.5 and slope > -2.0:
                left_slope_sum += slope
                left_intercept_sum += intercept
                left_count += 1
            elif slope > 0.5 and slope < 2.0:
                right_slope_sum += slope
                right_intercept_sum += intercept
                right_count += 1
        # Compute the average slope and intercept for left and right lanes
        if left_count > 0:
            left_slope = left_slope_sum / left_count
            left_intercept = left_intercept_sum / left_count
        else:
            left_slope = None
            left_intercept = None
        if right_count > 0:
            right_slope = right_slope_sum / right_count
            right_intercept = right_intercept_sum / right_count
        else:
            right_slope = None
            right_intercept = None

        # Compute the intersection point of the left and right lanes
        if left_slope is not None and right_slope is not None:
            x_intersect = (right_intercept - left_intercept) / (left_slope - right_slope)
            y_intersect = left_slope * x_intersect + left_intercept
            cv2.circle(img, (int(x_intersect), int(y_intersect)), 5, (0, 255, 0), 3)
        else:
            x_intersect = None
            y_intersect = None

        # Compute the steering angle
        if x_intersect is not None and y_intersect is not None:
            error = width/2 - x_intersect
            steering_angle = np.arctan(error * self.steering_gain)
            steering_angle = np.clip(steering_angle, -self.max_steering_angle, self.max_steering_angle)
        else:
            steering_angle = 0

        # Publish the steering angle
        self.steering_pub.publish(Float32(steering_angle))

        # Draw the lane lines and intersection point on the image
        if left_slope is not None:
            left_y1 = roi_height
            left_y2 = int(left_slope * width + left_intercept - roi_y_offset * left_slope)
            cv2.line(img, (0, left_y2), (width, left_y1), (0, 0, 255), 2)
        if right_slope is not None:
            right_y1 = roi_height
            right_y2 = int(right_slope * width + right_intercept - roi_y_offset * right_slope)
            cv2.line(img, (0, right_y2), (width, right_y1), (255, 0, 0), 2)

        # Publish the image
        cv2.imshow("Results", img)
        cv2.waitKey(30)
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(img, encoding='bgr8'))

if __name__ == '__main__':
    try:
        detector = LaneDetectorNode()
        rospy.spin()
        cv2.destroyAllWindows()
    except rospy.ROSInterruptException:
        pass
