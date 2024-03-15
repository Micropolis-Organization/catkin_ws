#!/usr/bin/env python

import sys
sys.path.append('/home/orin/catkin_ws/src/autonomous_steering/scripts')

import os, ctypes, platform

if platform.system() == "Linux":
     # Force libgomp to be loaded before other libraries consuming dynamic TLS (to avoid running out of STATIC_TLS)
     ctypes.cdll.LoadLibrary("libgomp.so.1")

import json
from frame_subscriber import OBDETWD_set_output_frames
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import torch as torch
import cv2
import numpy as np
import time
import math

sys.path.append(os.path.dirname(__file__))
from autonomous_utils.engine import BBoxVisualization
from autonomous_utils.yolo_classes import get_cls_dict
from midas.midas_net_custom import MidasNet_small
from midas.transforms import Resize, NormalizeImage, PrepareForNet
from torchvision.transforms import Compose
from midas.dpt_depth import DPTDepthModel

bridge = CvBridge()
img_topic = "/camera/Front_Camera"
person_pub = rospy.Publisher('person_depth', Float32, queue_size=10)
output_frame_bbox__pub = rospy.Publisher('output_frame_bbox', Image, queue_size=10)
output_frame_dmap__pub = rospy.Publisher('output_frame_dmap', Image, queue_size=10)
sign_pub = rospy.Publisher('sign_depth', Float32, queue_size=10)
person_number_pub = rospy.Publisher('persons_number', Float32, queue_size=10)
output_frame_dmap = np.zeros((480, 640, 3), dtype=np.uint8)
output_frame_bbox = np.zeros((480, 640, 3), dtype=np.uint8)

def OBDETWD_get_output_frames():
    global output_frame_bbox, output_frame_dmap
    output_frame_dmap = cv2.resize(np.array(output_frame_dmap), (640, 480), interpolation=cv2.INTER_AREA)
    output_frame_bbox = cv2.resize(np.array(output_frame_bbox), (640, 480), interpolation=cv2.INTER_AREA)
    return np.array(output_frame_bbox, dtype=np.uint8), np.array(output_frame_dmap, dtype=np.uint8)


min_depth, sign_depth, persons_number = 100, 0, 0

config_path = os.path.join(os.path.dirname(__file__), 'config/config.json')
with open(config_path) as config_file:
    config = json.load(config_file)

# Load a MiDas model for depth estimation
# model_type = "DPT_Large"     # MiDaS v3 - Large     (highest accuracy, slowest inference speed)
# model_type = "DPT_Hybrid"   # MiDaS v3 - Hybrid    (medium accuracy, medium inference speed)
model_type = "MiDaS_small"  # MiDaS v2.1 - Small   (lowest accuracy, highest inference speed)

# midas = torch.hub.load("intel-isl/MiDaS", model_type)
# Midas 2.1 Small
# midas_path = os.path.join(os.path.dirname(__file__), 'models/model-small-70d6b9c8.pt')
# midas = MidasNet_small(midas_path, features=64, backbone="efficientnet_lite3", exportable=True, non_negative=True, blocks={'expand': True})
# net_w, net_h = 256, 256
# resize_mode="upper_bound"
# normalization = NormalizeImage(
#     mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]
# )

# Midas 3 Hybird
midas_path = os.path.join(os.path.dirname(__file__), 'models/dpt_hybrid-midas-501f0c75.pt')
midas = DPTDepthModel(
            path=midas_path,
            backbone="vitb_rn50_384",
            non_negative=True,
        )
net_w, net_h = 384, 384
resize_mode="minimal"
normalization = NormalizeImage(mean=[0.5, 0.5, 0.5], std=[0.5, 0.5, 0.5])

# Move model to GPU if available
device = torch.device("cuda:0") if torch.cuda.is_available() else torch.device("cpu")
midas.to(device)
midas.eval()

# Load transforms to resize and normalize the image
# midas_transforms = torch.hub.load("intel-isl/MiDaS", "transforms")

# if model_type == "DPT_Large" or model_type == "DPT_Hybrid":
#     transform = midas_transforms.dpt_transform
# else:
#     transform = midas_transforms.small_transform

transform = Compose(
        [
            Resize(
                net_w,
                net_h,
                resize_target=None,
                keep_aspect_ratio=True,
                ensure_multiple_of=32,
                resize_method=resize_mode,
                image_interpolation_method=cv2.INTER_CUBIC,
            ),
            normalization,
            PrepareForNet(),
        ]
    )

optimize = True
if optimize == True:
    if device == torch.device("cuda"):
        midas = midas.to(memory_format=torch.channels_last)
        midas = midas.half()

# Init Yolo Models
cls_dict = get_cls_dict('coco')
vis = BBoxVisualization(cls_dict)
yolo5_path = os.path.join(os.path.dirname(__file__), 'models/yolov5m.engine')
# yolo5_model = torch.hub.load('ultralytics/yolov5', 'custom', path=yolo5_path)
yolo5_model = torch.hub.load('/home/microspot/m2p_ws/src/camera_stream/scripts/hubconf.py', 'custom', path=str(yolo5_path), source='local')


def callback(msg):
    frame = bridge.imgmsg_to_cv2(msg, "bgr8")
    frame = frame[:-70,:-150,:]
    process_img(frame)


def process_img(frame):
    global output_frame_bbox, output_frame_dmap
    start = time.time()
    img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    # Apply input transforms
    img_input = transform({"image": img})["image"]

    # Prediction and resize to original resolution
    with torch.no_grad():
        sample = torch.from_numpy(img_input).to(device).unsqueeze(0)

        prediction = midas.forward(sample)

        prediction = torch.nn.functional.interpolate(
            prediction.unsqueeze(1),
            size=img.shape[:2],
            mode="bicubic",
            align_corners=False,
        ).squeeze()
    depth_map = prediction.cpu().numpy()
    depth_map = cv2.normalize(depth_map, None, 0, 1, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_64F)

    results = yolo5_model(img)
    df = results.pandas().xyxy[0]

    # calculate objects depths
    boxes, confs, classes, names, depths = objects_depths(df, depth_map)
    
    global min_depth, sign_depth
    min_depth, sign_depth = 100, 100
    persons_depths, signs_depths = list(), list()
    if len(depths):
        for index, cls in enumerate(classes):
            if cls == 0:
                persons_depths.append(depths[index])
            elif cls == 11:
                signs_depths.append(depths[index])
        if(persons_depths):
            min_depth = min(persons_depths)
        if(signs_depths):
            sign_depth = min(signs_depths)

        img, _visualize_time = vis.draw_bboxes(img, boxes, confs, classes, depths)

    end = time.time()
    total_time = end - start

    fps = 1 / total_time

    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

    depth_map = (depth_map * 255).astype(np.uint8)
    depth_map = cv2.applyColorMap(depth_map, cv2.COLORMAP_MAGMA)
    cv2.putText(img, f'FPS: {int(fps)}', (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 0), 2)
    # cv2.imshow('Image', img)
    # cv2.imshow('Depth Map', depth_map)
    # cv2.waitKey(1)
    output_frame_dmap = cv2.resize(np.array(depth_map), (640, 480), interpolation=cv2.INTER_AREA)
    output_frame_bbox = cv2.resize(np.array(img), (640, 480), interpolation=cv2.INTER_AREA)
    OBDETWD_set_output_frames(img, depth_map)

def norm_distance(distance):
    if distance > 0.4:
        return 0
    # elif 0.3 <= distance:
    #     return (1 - distance)* 10
    else:
        return 100

def norm_sign_distance(distance):
    if distance > 0.2:
        return 0
    # elif 0.3 <= distance:
    #     return (1 - distance)* 10
    else:
        return 100

def objects_depths(df, img):
    boxes, confs, classes, names, depths = list(), list(), list(), list(), list()
    global persons_number
    persons_number = 0 
    for index, row in df.iterrows():
        if row['class'] == 0 and row['confidence'] >= 0.6:
            box, cls, confidence, name, depth_mean = object_depth(row, img)
            boxes.append(box)
            confs.append(confidence)
            classes.append(cls)
            names.append(name)
            depth_meter = (5.7966 - 6.41642 * round(float(depth_mean), 3))
            x = round(float(depth_mean), 3)
            depth_meter = (8.77939 * x * x) - 13.5359 * x + 6.16954
            depths.append(norm_distance(depth_mean))
            # global persons_number
            persons_number += 1

        if row['class'] == 11 and row['confidence'] >= 0.6:
            box, cls, confidence, name, depth_mean = object_depth(row, img)
            boxes.append(box)
            confs.append(confidence)
            classes.append(cls)
            names.append(name)
            depth_meter = (5.7966 - 6.41642 * round(float(depth_mean), 3))
            x = round(float(depth_mean), 3)
            depth_meter = (8.77939 * x * x) - 13.5359 * x + 6.16954
            depths.append(norm_sign_distance(depth_mean))

    return boxes, confs, classes, names, depths


def object_depth(row, img):
    x_min, y_min, x_max, y_max = row['xmin'], row['ymin'], row['xmax'], row['ymax']
    x_cut = int((x_max - x_min) / 10)
    y_cut = int((y_max - y_min) / 10)
    rect = img[int((y_min + y_cut)):int(y_max), int((x_min + x_cut)):int((x_max - x_cut))]
    temp = rect.flatten()
    try:
        temp = sorted(list(temp))
        n_max = temp[-int(len(temp) / 4):]
    except Exception as e:
        n_max = temp

    depth_mean = round(float(np.mean(n_max)), 5)
    box = (int(x_min), int(y_min), int(x_max), int(y_max))
    cls = row['class']
    confidence = row['confidence']
    name = row['name']
    return box, cls, confidence, name, depth_mean


def depth_estimation_subscriber():
    rospy.init_node('frame_subscriber', anonymous=True)

    rospy.Subscriber(img_topic, Image, callback)
    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        global min_depth, sign_depth, persons_number, output_frame_dmap, output_frame_bbox
        person_pub.publish(min_depth)
        sign_pub.publish(sign_depth)
        person_number_pub.publish(persons_number)

        output_frame_dmap__pub.publish(bridge.cv2_to_imgmsg(output_frame_dmap, "bgr8"))
        output_frame_bbox__pub.publish(bridge.cv2_to_imgmsg(output_frame_bbox, "bgr8"))
        rate.sleep()


if __name__ == '__main__':
    depth_estimation_subscriber()
