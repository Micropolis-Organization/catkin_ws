#!/usr/bin/env python3

import time
import cv2
import os
from cv2 import COLOR_BAYER_BG2BGR
from cv2 import COLOR_GRAY2BGR
import matplotlib.pyplot as plt
from calibration_utils import calibrate_camera, undistort
from binarization_utils import binarize
from perspective_utils import birdeye
from line_utils import get_fits_by_sliding_windows, draw_back_onto_the_road, Line, get_fits_by_previous_fits, get_steering_angle
# from moviepy.editor import VideoFileClip

import numpy as np
from globals import xm_per_pix, time_window
processed_frames = 0

 
curveList = []
avgVal=100
output_frame_binarized = None
output_frame_road_filled = None
output_frame_birdseye = None 
output_frame_steering = None


def AutoSteer_get_output_frames():
    global output_frame_binarized, output_frame_road_filled, output_frame_birdseye, output_frame_steering

    output_frame_road_filled = cv2.cvtColor(output_frame_road_filled, COLOR_GRAY2BGR) 
    output_frame_binarized = cv2.cvtColor(output_frame_binarized, COLOR_GRAY2BGR)
    output_frame_birdseye = cv2.cvtColor(output_frame_birdseye, COLOR_GRAY2BGR)

    return np.array(output_frame_binarized), np.array(output_frame_road_filled), np.array(output_frame_birdseye), np.array(output_frame_steering)

def prepare_out_blend_frame(blend_on_road, img_binary, img_birdeye, img_fit, line_lt, line_rt, offset_meter, degrees, curve_direction):
    """
    Prepare the final pretty pretty output blend, given all intermediate pipeline images

    :param blend_on_road: color image of lane blend onto the road
    :param img_binary: thresholded binary image
    :param img_birdeye: bird's eye view of the thresholded binary image
    :param img_fit: bird's eye view with detected lane-lines highlighted
    :param line_lt: detected left lane-line
    :param line_rt: detected right lane-line
    :param offset_meter: offset from the center of the lane
    :return: pretty blend with all images and stuff stitched
    """
    h, w = blend_on_road.shape[:2]

    thumb_ratio = 0.2
    thumb_h, thumb_w = int(thumb_ratio * h), int(thumb_ratio * w)

    off_x, off_y = 20, 15

    # add a gray rectangle to highlight the upper area
    mask = blend_on_road.copy()
    mask = cv2.rectangle(mask, pt1=(0, 0), pt2=(w, thumb_h+2*off_y), color=(0, 0, 0), thickness=cv2.FILLED)
    blend_on_road = cv2.addWeighted(src1=mask, alpha=0.2, src2=blend_on_road, beta=0.8, gamma=0)

    # add thumbnail of binary image
    thumb_binary = cv2.resize(img_binary, dsize=(thumb_w, thumb_h))
    thumb_binary = np.dstack([thumb_binary, thumb_binary, thumb_binary]) * 255
    blend_on_road[off_y:thumb_h+off_y, off_x:off_x+thumb_w, :] = thumb_binary

    # add thumbnail of bird's eye view
    thumb_birdeye = cv2.resize(img_birdeye, dsize=(thumb_w, thumb_h))
    thumb_birdeye = np.dstack([thumb_birdeye, thumb_birdeye, thumb_birdeye]) * 255
    blend_on_road[off_y:thumb_h+off_y, 2*off_x+thumb_w:2*(off_x+thumb_w), :] = thumb_birdeye

    # add thumbnail of bird's eye view (lane-line highlighted)
    thumb_img_fit = cv2.resize(img_fit, dsize=(thumb_w, thumb_h))
    blend_on_road[off_y:thumb_h+off_y, 3*off_x+2*thumb_w:3*(off_x+thumb_w), :] = thumb_img_fit

    # add text (curvature and offset info) on the upper right of the blend
    mean_curvature_meter = np.mean([line_lt.curvature_meter, line_rt.curvature_meter])
    

      
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(blend_on_road, 'Curvature radius: {:.02f}m'.format(mean_curvature_meter), (460, 30), font, 0.4, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(blend_on_road, 'Offset from center: {:.02f}m'.format(offset_meter), (460, 60), font, 0.4, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(blend_on_road, 'Steering: {} {:.02f}deg'.format(curve_direction, degrees), (460, 90), font, 0.4, (255, 255, 255), 1, cv2.LINE_AA)

    return blend_on_road


def compute_offset_from_center(line_lt, line_rt, frame_width):
    """
    Compute offset from center of the inferred lane.
    The offset from the lane center can be computed under the hypothesis that the camera is fixed
    and mounted in the midpoint of the car roof. In this case, we can approximate the car's deviation
    from the lane center as the distance between the center of the image and the midpoint at the bottom
    of the image of the two lane-lines detected.

    :param line_lt: detected left lane-line
    :param line_rt: detected right lane-line
    :param frame_width: width of the undistorted frame
    :return: inferred offset, curvature radius in meter, steering degrees, curveing direction
    """
    if line_lt.detected and line_rt.detected:
        line_lt_bottom = np.mean(line_lt.all_x[line_lt.all_y > 0.95 * line_lt.all_y.max()])
        line_rt_bottom = np.mean(line_rt.all_x[line_rt.all_y > 0.95 * line_rt.all_y.max()])
        lane_width = line_rt_bottom - line_lt_bottom
        midpoint = frame_width / 2
        offset_pix = (line_lt_bottom + lane_width / 2) - midpoint
        offset_meter = xm_per_pix * offset_pix 
        mean_curvature_meter = np.mean([line_lt.curvature_meter, line_rt.curvature_meter])
        # center_fit = line_lt.recent_fits_pixel + line_rt.recent_fits_pixel

        print("")
        print("")
        print(f"line_rt.curvature_meter: {line_rt.curvature_meter}")
        print(f"line_rt.last_fit_pixel[-1]: {line_rt.last_fit_pixel[0]}")
        print("")
        print("")
        print("=========================================")
        

        if line_rt.last_fit_pixel[-1] < 0:
            curve_direction = 'Left Curve'
            radius = 5729.57795 / line_rt.curvature_meter
        elif line_rt.last_fit_pixel[-1] > 0:
            curve_direction = 'Right Curve'
            radius = -5729.57795 / line_rt.curvature_meter
        else:
            curve_direction = 'Straight'
            radius = 5729.57795 / line_rt.curvature_meter

        degrees = radius * 1
        
        if degrees > 40:
            degrees = 40
        elif degrees < -40:
            degrees = -40  
    else:
        mean_curvature_meter = 0
        offset_meter = -1
        degrees = 0
        curve_direction = "Straight"
    return mean_curvature_meter, offset_meter, degrees * 3.14159 / 180, curve_direction



def fill_the_road(img):
    binary = img.copy()
    points = []
    h, w = binary.shape
    for row in range((h // 3) * 2):
        row = row + (h // 3)
        i, j = (0, w - 1)
        left, right = (False, False)
        while i <= j:
            if binary[row, i] and not left:
                left = True
                left_pt = [i, row]

            if binary[row, j] and not right:
                right = True
                right_pt = [j, row]

            if left and right:
                points.append(left_pt)
                points.append(right_pt)
                break
            i = i + 1
            j = j - 1
    
    if len(points):
        cv2.fillPoly(binary, np.array([points], dtype=np.int32), 255)
    # cv2.imshow("Road", binary)
    # cv2.waitKey(30)
    return binary


def getHistogram(img,minPer=0.1,display= False,region=1):
 
    if region ==1:
        histValues = np.sum(img, axis=0)
    else:
        histValues = np.sum(img[img.shape[0]//region:,:], axis=0)
 
    #print(histValues)
    maxValue = np.max(histValues)
    minValue = minPer*maxValue
 
    indexArray = np.where(histValues >= minValue)
    basePoint = int(np.average(indexArray))
    #print(basePoint)
 
    if display:
        imgHist = np.zeros((img.shape[0],img.shape[1],3),np.uint8)
        for x,intensity in enumerate(histValues):
            cv2.line(imgHist,(x,img.shape[0]),(x,int(img.shape[0]-intensity//255//region)),(255,0,255),1)
            cv2.circle(imgHist,(basePoint,img.shape[0]),20,(0,255,255),cv2.FILLED)
        return basePoint,imgHist
 
    return basePoint
 


def process_pipeline_with_da(img_binary, is_binarized = False, is_road_filled = False, verbose=False):
    global output_frame_binarized, output_frame_road_filled, output_frame_birdseye, output_frame_steering
    start_t = time.time()
    # binarize the frame s.t. lane lines are highlighted as much as possible
    if not is_binarized:
        img_binary = binarize(img_binary, verbose=False)
    output_frame_binarized = img_binary * 255
    if verbose:
        cv2.imshow("Binarization", img_binary * 255)


    bin_filled = img_binary
    if not is_road_filled:
        bin_filled = fill_the_road(img_binary)
    output_frame_road_filled = bin_filled
    if verbose:
         cv2.imshow("Filled", bin_filled)

    img_birdeye, M, Minv = birdeye(bin_filled, verbose=False)
    output_frame_birdseye = img_birdeye
    if verbose:
        cv2.imshow("Bird", img_birdeye)
        cv2.waitKey(30)
    

    #### STEP 3
    middlePoint,imgHist = getHistogram(img_birdeye,display=True,minPer=0.5,region=4)
    curveAveragePoint, imgHist = getHistogram(img_birdeye, display=True, minPer=0.9)
    curveRaw = curveAveragePoint - middlePoint

    
    if curveRaw < -40:
        curveRaw = -40
    if curveRaw > 40:
        curveRaw = 40

    output_frame_steering = imgHist
    end_t = time.time()
    # print("=================================================")
    # print("{}".format(end_t - start_t))
    # print("=================================================")

    return float(curveRaw * 3.14159 / 180), imgHist
    
def process_pipeline(frame, line_lt, line_rt, mtx, dist, keep_state=True, is_binarized=False):
    """
    Apply whole lane detection pipeline to an input color frame.
    :param frame: input color frame
    :param keep_state: if True, lane-line state is conserved (this permits to average results)
    :return: output computed steering variables
    """
    global output_frame_binarized, output_frame_road_filled, output_frame_birdseye, output_frame_steering, processed_frames

    # frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    
    # binarize the frame s.t. lane lines are highlighted as much as possible
    img_binary = frame
    if not is_binarized:
        img_binary = binarize(img_binary, verbose=False)
    output_frame_binarized = img_binary * 255
    

    output_frame_road_filled = fill_the_road(img_binary)
    # compute perspective transform to obtain bird's eye view
    img_birdeye, M, Minv = birdeye(img_binary, verbose=False)
    output_frame_birdseye = img_birdeye

    # fit 2-degree polynomial curve onto lane lines found
    if processed_frames > 0 and keep_state and line_lt.detected and line_rt.detected:
        line_lt, line_rt, img_fit = get_fits_by_previous_fits(img_birdeye, line_lt, line_rt, verbose=False)
    else:
        line_lt, line_rt, img_fit = get_fits_by_sliding_windows(img_birdeye, line_lt, line_rt, n_windows=18, verbose=False)

    mean_curvature_meter, offset_meter, steering_angle_deg, curve_direction = compute_offset_from_center(line_lt, line_rt, frame_width=frame.shape[1])

    blend_on_road = draw_back_onto_the_road(frame, Minv, line_lt, line_rt, keep_state)

    blend_output = prepare_out_blend_frame(blend_on_road, img_binary, img_birdeye, img_fit, line_lt, line_rt, offset_meter, steering_angle_deg, curve_direction)
    output_frame_steering = blend_output


    processed_frames = processed_frames + 1

    return mean_curvature_meter, offset_meter, steering_angle_deg, curve_direction, blend_output
if __name__ == '__main__':
    pass
   
