#!/usr/bin/env python
import argparse
import os, sys
import shutil
import time
from pathlib import Path
from cv2 import COLOR_GRAY2BGR
import imageio
import cv2
import torch
import torch.backends.cudnn as cudnn
from numpy import random
import scipy.special
import numpy as np
import torchvision.transforms as transforms
import PIL.Image as image
import rospy
from std_msgs.msg import String
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import time
sys.path.append(os.path.dirname(__file__))
from lib.config import cfg
from lib.config import update_config
from lib.utils.utils import create_logger, select_device, time_synchronized
from lib.models import get_net
from lib.dataset import LoadImages, LoadStreams
from lib.core.general import non_max_suppression, scale_coords
from lib.utils import plot_one_box,show_seg_result
from lib.core.function import AverageMeter
from lib.core.postprocess import morphological_process, connect_lane
from tqdm import tqdm



bridge = CvBridge()
img_topic = "/camera/image_raw"
pub = rospy.Publisher('depth', String, queue_size=10)


normalize = transforms.Normalize(
        mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]
    )

transform=transforms.Compose([
            transforms.ToTensor(),
            normalize,
        ])

logger, _, _ = create_logger(cfg, cfg.LOG_DIR, 'demo')

device = select_device(logger,'0')
   
half = device.type != 'cpu'  # half precision only supported on CUDA

 # Load model
model = get_net(cfg)
weights_path = os.path.join(os.path.dirname(__file__), 'weights/End-to-end.pth')
checkpoint = torch.load(weights_path, map_location= device)
model.load_state_dict(checkpoint['state_dict'])
model = model.to(device)
if half:
    model.half()  # to FP16


# Run inference
t0 = time.time()

vid_path, vid_writer = None, None
img = torch.zeros((1, 3, 640, 640), device=device)  # init img
_ = model(img.half() if half else img) if device.type != 'cpu' else None  # run once
model.eval()

inf_time = AverageMeter()
nms_time = AverageMeter()


def callback(msg):
    
    frame = bridge.imgmsg_to_cv2(msg, "bgr8")
    process_img(frame)


def filter(input_image):

    hsv = cv2.cvtColor(input_image,cv2.COLOR_BGR2HSV)

    # Create the HSV range for the blue ink:
    # [128, 255, 255], [90, 50, 70]
    lowerValues = np.array([0, 0, 231])
    upperValues = np.array([180, 18, 255])

    # Get binary mask of the blue ink:
    mask = cv2.inRange(hsv, lowerValues, upperValues)
    input_image[mask>0]=(128,128,128)
    

    ret, mask = cv2.threshold(input_image[:, :,2], 200, 255, cv2.THRESH_BINARY)

    mask3 = np.zeros_like(input_image)
    mask3[:, :, 0] = mask
    mask3[:, :, 1] = mask
    mask3[:, :, 2] = mask

    # extracting `orange` region using `biteise_and`
    orange = cv2.bitwise_and(input_image, mask3)
    
    print(orange.shape)
    # print(input_image.shape)
    # cv2.imshow('img',orange)
    # cv2.waitKey()
    # output = cv2.cvtColor(input_image, cv2.COLOR_HSV2RGB)
    
    return orange


def process_img(frame):
    frame = cv2.flip(frame, 1)
    
    frame = cv2.resize(frame,(640, 480))
    img = transform(frame).to(device)
    img = img.half() if half else img.float()  # uint8 to fp16/32
    if img.ndimension() == 3:
        img = img.unsqueeze(0)
    t1 = time_synchronized()
    det_out, da_seg_out,ll_seg_out= model(img)
    t2 = time_synchronized()
    inf_out, _ = det_out
    inf_time.update(t2-t1,img.size(0))  
    _, _, height, width = img.shape
    pad_w, pad_h = (0,0)
    pad_w = int(pad_w)
    pad_h = int(pad_h)
    ratio = 1
    da_predict = da_seg_out[:, :, pad_h:(height-pad_h),pad_w:(width-pad_w)]
    da_seg_mask = torch.nn.functional.interpolate(da_predict, scale_factor=int(1/ratio), mode='bilinear')
    _, da_seg_mask = torch.max(da_seg_mask, 1)
    da_seg_mask = da_seg_mask.int().squeeze().cpu().numpy()
    
    kernel = np.ones((9, 9), np.uint8)
    closing = cv2.morphologyEx(da_seg_mask.astype(np.uint8), cv2.MORPH_OPEN, kernel)
    closing = cv2.morphologyEx(closing.astype(np.uint8), cv2.MORPH_OPEN, kernel)

    drivable_area = closing.astype(np.uint8) * 255
    drivable_area = region_of_interest(drivable_area, np.array([[[0,480],[640,480],[640, 400],[480,340],[160,340],[0,400]]], dtype=np.int32))
    # apply a light morphology to "fill the gaps" in the binary image
    
    drivable_area = cv2.cvtColor(drivable_area, COLOR_GRAY2BGR)
    
    cv2.imshow('da1',drivable_area)
    # cv2.imshow('closing',closing)

    ll_predict = ll_seg_out[:, :,pad_h:(height-pad_h),pad_w:(width-pad_w)]
    ll_seg_mask = torch.nn.functional.interpolate(ll_predict, scale_factor=int(1/ratio), mode='bilinear')
    _, ll_seg_mask = torch.max(ll_seg_mask, 1)
    ll_seg_mask = ll_seg_mask.int().squeeze().cpu().numpy()
    lines = ll_seg_mask.astype(np.uint8) * 255
    lines = region_of_interest(lines, np.array([[[0,480],[640,480],[640, 400],[480,340],[160,340],[0,400]]], dtype=np.int32))
    lines = cv2.cvtColor(lines, COLOR_GRAY2BGR)
    
    lines = show_seg_result(frame, (da_seg_mask, ll_seg_mask), _, _, is_demo=True)
    cv2.imshow('Image', frame)
    cv2.waitKey(1)
    return drivable_area, lines


def region_of_interest(img, vertices):
        
    mask = np.zeros_like(img)   
    cv2.fillPoly(mask, vertices, color=255)
    
    masked_image = cv2.bitwise_and(img, mask)
    return masked_image

def line_detection_subscriber():
    rospy.init_node('frame_subscriber', anonymous=True)
    rospy.Subscriber(img_topic, Image, callback)
    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        global min_depth
        pub.publish(str(''))
        rate.sleep()


if __name__ == '__main__':

    
    line_detection_subscriber()
