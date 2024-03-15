#!/usr/bin/env python3
import sys
sys.path.insert(1, '/home/microspot/m2p_ws/src/camera_stream/scripts')

from cv2 import VideoWriter
from calibration_utils import calibrate_camera, undistort
from line_utils import Line
from main import AutoSteer_get_output_frames, process_pipeline, process_pipeline_with_da  
import rospy
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Float32, String, Bool, Int16
from cv_bridge import CvBridge, CvBridgeError
import cv2
from globals import xm_per_pix, time_window
import os
import time
from cv2 import COLOR_GRAY2BGR
import numpy as np


bridge = CvBridge()
scale = 0.5
do_pre_undistortion = False
steering_angle = 0.0
curvature_radius = 0
offset_meter = -1
curve_direction = "Straight"

ret = []
mtx = []
dist = []
rvecs = []
tvecs = []

line_lt = Line(buffer_len=time_window)  # line on the left of the lane
line_rt = Line(buffer_len=time_window)  # line on the right of the lane

closed = False

img_topic = ''
camera_calibration_dir = ''
video_source = ''
mode = ''
recording = False
videoWriter = None
output_frame_dmap = np.zeros((480, 640, 3), dtype=np.uint16)
output_frame_bbox = np.zeros((480, 640, 3), dtype=np.uint16)


def OBDETWD_set_output_frames(img, depth):
    global output_frame_bbox, output_frame_dmap
    output_frame_dmap = cv2.resize(np.array(img), (640, 480), interpolation=cv2.INTER_AREA)
    output_frame_bbox = cv2.resize(np.array(depth), (640, 480), interpolation=cv2.INTER_AREA)
    return np.array(output_frame_bbox), np.array(output_frame_dmap)



def output_frame_bbox_cb(msg):
    global output_frame_bbox
    output_frame_bbox = bridge.imgmsg_to_cv2(msg, "bgr8")

def output_frame_dmap_cb(msg):
    global output_frame_dmap
    output_frame_dmap = bridge.imgmsg_to_cv2(msg, "bgr8")


def img_callback(msg):
    global do_pre_undistortion
    # rospy.loginfo("Image received ...")
    g_cv2_img = bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
    process_img(g_cv2_img, _undistort=do_pre_undistortion)


def process_img(cv2_img, _undistort = True):
    global mtx, dist, line_lt, line_rt, curvature_radius, offset_meter, steering_angle, curve_direction, scale, videoWriter, closed, output_frame_bbox, output_frame_dmap
    resized = cv2.resize(cv2_img, (int(1280 * scale), int(960 * scale)), interpolation=cv2.INTER_AREA)
    if _undistort:
        resized = undistort(resized, mtx, dist, verbose=False)

    # ======== DL Method
    # drivable_area, lanes_binarized = get_da_and_lanes(resized)
    # cv2.imshow("lanes_binarized", drivable_area)
    # ========
     
    # curvature_radius, offset_meter, steering_angle, curve_direction, blend_output = process_pipeline(resized, line_lt, line_rt, mtx, dist, False)
    # steering_angle, blend_output = process_pipeline_with_da(drivable_area, is_binarized=True, is_road_filled=True)
    steering_angle, blend_output = process_pipeline_with_da(resized, is_binarized=False, is_road_filled=False, verbose=False)
    
    # Visualaization
    AS_output = AutoSteer_get_output_frames()
    OBDETWD_output = [output_frame_bbox, output_frame_dmap]
    first_row = np.hstack((AS_output[0], AS_output[2], OBDETWD_output[0]))
    second_row = np.hstack((AS_output[1], AS_output[3], OBDETWD_output[1]))
    output_frame_final = np.vstack((first_row, second_row))
    if recording:
        videoWriter.write(np.uint8(output_frame_final))
    else:
        cv2.imshow("output_frame_final", np.uint8(output_frame_final))
        if cv2.waitKey(1) & 0xFF == ord('q'):
                closed = True

    
def init_pub_sub():
    global ret, mtx, dist, rvecs, tvecs, camera_calibration_dir, video_source, mode, img_topic, videoWriter, closed
    
    for arg in sys.argv:
        if "--camera_topic" in arg:
            img_topic = arg.split("=")[1]

    rospy.init_node('autonomous_steering', anonymous=True)
    
    camera_calibration_dir = rospy.get_param('~camera_calibration_dir', default="")
    video_source = rospy.get_param('~video_source', default="")
    mode = rospy.get_param('~mode', default="live") #'video' or 'camera'
    img_topic = rospy.get_param('~img_topic',default="/camera/image_raw")
    if img_topic == "" and mode != 'video':
        return
    
    videoWriter = cv2.VideoWriter('{}/outpy.avi'.format(camera_calibration_dir), cv2.VideoWriter_fourcc(*'XVID'), 20, (640 * 3, 480 * 2))

    ret, mtx, dist, rvecs, tvecs = calibrate_camera(calib_images_dir=camera_calibration_dir)
    
    if mode != 'video':
        rospy.Subscriber(img_topic, CompressedImage, img_callback)
    rospy.Subscriber('output_frame_bbox', Image, output_frame_bbox_cb)
    rospy.Subscriber('output_frame_dmap', Image, output_frame_dmap_cb)
    angle_pub = rospy.Publisher('theta', Float32, queue_size=30)
    radius_pub = rospy.Publisher('curvature_radius', Float32, queue_size=30)
    offset_pub = rospy.Publisher('offset_from_center', Float32, queue_size=30)
    dir_pub = rospy.Publisher('curve_direction', String, queue_size=30)
    left_detected = rospy.Publisher('left_lane_detected', Bool, queue_size=30)
    right_detected = rospy.Publisher('right_lane_detected', Bool, queue_size=30)
    rate = rospy.Rate(10) # 20hz

    cap = cv2.VideoCapture(video_source)

    smoothed_angle = 0.0000000001

    while not rospy.is_shutdown():
        if mode == 'video':
            ret, frame = cap.read()
            if ret is not True: 
                print("No Frames!")
                break
            process_img(frame)
        if closed:
            break
        smoothed_angle += 0.2 * pow(abs((steering_angle - smoothed_angle)), 2.0 / 3.0) * (steering_angle - smoothed_angle) / abs(steering_angle - smoothed_angle)
        angle_pub.publish(smoothed_angle)
        radius_pub.publish(curvature_radius)
        offset_pub.publish(offset_meter)
        dir_pub.publish(curve_direction)
        left_detected.publish(True)
        right_detected.publish(True)
        rate.sleep()

    cap.release()
    videoWriter.release() 
    cv2.destroyAllWindows()


if __name__ == '__main__':
    
    
    
    try:
        init_pub_sub()

    except rospy.ROSInterruptException:
        pass


