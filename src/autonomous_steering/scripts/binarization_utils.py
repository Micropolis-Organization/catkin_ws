#!/usr/bin/env python3

import cv2
import numpy as np
import glob
import matplotlib.pyplot as plt


# selected threshold to highlight yellow lines
# yellow_HSV_th_min = np.array([0 , 112 , 112])
# yellow_HSV_th_max = np.array([120, 255 ,255])

yellow_HSV_th_min = np.array([10 , 70 , 122])
yellow_HSV_th_max = np.array([35, 255, 255])



def thresh_frame_in_HSV(frame, min_values, max_values, verbose=False):
    """
    Threshold a color frame in HSV space
    """
    HSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    min_th_ok = np.all(HSV > min_values, axis=2)
    max_th_ok = np.all(HSV < max_values, axis=2)

    out = np.logical_and(min_th_ok, max_th_ok)

    if verbose:
        plt.imshow(out, cmap='gray')
        plt.show()

    return out


def thresh_frame_sobel(frame, kernel_size):
    """
    Apply Sobel edge detection to an input frame, then threshold the result
    """
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    sobel_x = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=kernel_size)
    sobel_y = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=kernel_size)

    sobel_mag = np.sqrt(sobel_x ** 2 + sobel_y ** 2)
    sobel_mag = np.uint8(sobel_mag / np.max(sobel_mag) * 255)

    _, sobel_mag = cv2.threshold(sobel_mag, 70, 1, cv2.THRESH_BINARY)

    return np.invert(sobel_mag.astype(bool))


def get_binary_from_equalized_grayscale(frame):
    """
    Apply histogram equalization to an input frame, threshold it and return the (binary) result.
    """
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    eq_global = cv2.equalizeHist(gray)

    _, th = cv2.threshold(eq_global, thresh=250, maxval=255, type=cv2.THRESH_BINARY)

    return th


def get_orangish_areas(img):
    green = img[...,1]

    mask = cv2.threshold(green, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)[1]

    Orange = cv2.bitwise_and(img, img, mask = mask)

    return Orange

def filter_contours(frame, binarized):
    contours, hierarchy = cv2.findContours((binarized*255).astype(np.uint8), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours = sorted(contours, key=cv2.contourArea, reverse=True)
    frame_out = np.zeros_like(frame)
    if len(contours) > 0:
        for cnt in contours:
            arc = cv2.arcLength(cnt, True)
            rect = cv2.minAreaRect(cnt)
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            h, w = rect[1][1], rect[1][0]
            if w and h and (h / w > 2 or w / h > 2):
                cv2.drawContours(frame_out, [cnt], 0, (255,255,255), 3)
            # cv2.imshow("Cnt", frame_out)
            # cv2.waitKey(0)
    frame_out = cv2.cvtColor(frame_out, cv2.COLOR_BGR2GRAY) // 255

    return frame_out


def binarize(img, verbose=False):
    """
    Convert an input frame to a binary image which highlight as most as possible the lane-lines.

    :param img: input color frame
    :param verbose: if True, show intermediate results
    :return: binarized frame
    """
    h, w = img.shape[:2]

    binary = np.zeros(shape=(h, w), dtype=np.uint8)

    # highlight yellow lines by threshold in HSV color space
    HSV_yellow_mask = thresh_frame_in_HSV(img, yellow_HSV_th_min, yellow_HSV_th_max, verbose=False)
    binary = np.logical_or(binary, HSV_yellow_mask)
    cnt_mask = filter_contours(img, binary)
    binary = np.bitwise_and(binary, cnt_mask)

    # highlight white lines by thresholding the equalized frame
    # eq_white_mask = get_binary_from_equalized_grayscale(img)
    # binary = np.logical_or(binary, eq_white_mask)

    # get Sobel binary mask (thresholded gradients)
    # sobel_mask = thresh_frame_sobel(img, kernel_size=9)
    # binary = np.logical_and(binary, sobel_mask)

    # apply a light morphology to "fill the gaps" in the binary image
    kernel = np.ones((3, 3), np.uint8)
    closing = cv2.morphologyEx(binary.astype(np.uint8), cv2.MORPH_CLOSE, kernel)

    kernel = np.ones((5, 5), np.uint8)
    closing = cv2.morphologyEx(closing.astype(np.uint8), cv2.MORPH_CLOSE, kernel)

    # kernel = np.ones((3, 3), np.uint8)
    # closing = cv2.morphologyEx(closing.astype(np.uint8), cv2.MORPH_CLOSE, kernel)

    # kernel = np.ones((9, 9), np.uint8)
    # closing = cv2.morphologyEx(closing.astype(np.uint8), cv2.MORPH_CLOSE, kernel)


    if verbose:
        f, ax = plt.subplots(2, 3)
        f.set_facecolor('white')
        ax[0, 0].imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
        ax[0, 0].set_title('input_frame')
        ax[0, 0].set_axis_off()
        ax[0, 0].set_axis_bgcolor('red')
        ax[0, 1].imshow(eq_white_mask, cmap='gray')
        ax[0, 1].set_title('white mask')
        ax[0, 1].set_axis_off()

        ax[0, 2].imshow(HSV_yellow_mask, cmap='gray')
        ax[0, 2].set_title('yellow mask')
        ax[0, 2].set_axis_off()

        ax[1, 0].imshow(sobel_mask, cmap='gray')
        ax[1, 0].set_title('sobel mask')
        ax[1, 0].set_axis_off()

        ax[1, 1].imshow(binary, cmap='gray')
        ax[1, 1].set_title('before closure')
        ax[1, 1].set_axis_off()

        ax[1, 2].imshow(closing, cmap='gray')
        ax[1, 2].set_title('after closure')
        ax[1, 2].set_axis_off()
        plt.show()

    return closing


if __name__ == '__main__':

    test_images = glob.glob('test_images/*.jpg')
    for test_image in test_images:
        img = cv2.imread(test_image)
        binarize(img=img, verbose=True)
