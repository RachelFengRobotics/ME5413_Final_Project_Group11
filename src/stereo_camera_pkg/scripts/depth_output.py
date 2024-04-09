# -*- coding: utf-8 -*-
import cv2
import numpy as np
import camera_config

# Create window for depth visualization
cv2.namedWindow("depth")
cv2.moveWindow('depth', 600, 0)

# Create trackbars for parameter adjustment
cv2.createTrackbar('num', 'depth', 3, 10, lambda x: None)
cv2.createTrackbar('blockSize', 'depth', 7, 255, lambda x: None)

def callbackFunc(e, x, y, f, p):
    if e == cv2.EVENT_LBUTTONDOWN:
        print(threeD[y][x])

# Set mouse callback function
cv2.setMouseCallback("depth", callbackFunc, None)

imgL = np.zeros((480, 640, 3), np.uint8)
imgR = np.zeros((480, 640, 3), np.uint8)
query_points = [(100, 150)]# query_point input is here !

# Adjust parameters using trackbars
num = cv2.getTrackbarPos("num", "depth")
blockSize = cv2.getTrackbarPos("blockSize", "depth")
if blockSize % 2 == 0:
    blockSize += 1
if blockSize < 5:
    blockSize = 5

# Map input is here!
gray_l = cv2.imread('left.png', 0)
gray_r = cv2.imread('right.png', 0)
gray_l = cv2.pyrDown(gray_l)
gray_r = cv2.pyrDown(gray_r)

# StereoSGBM settings
window_size = 1
min_disp = 16
num_disp = 112 - min_disp
stereo = cv2.StereoSGBM_create(
    minDisparity=min_disp,
    numDisparities=num_disp,
    blockSize=16,
    P1=8 * 3 * window_size**2,
    P2=32 * 3 * window_size**2,
    disp12MaxDiff=1,
    uniquenessRatio=10,
    speckleWindowSize=100,
    speckleRange=32,
)

# Use StereoBM with adjusted parameters
stereo = cv2.StereoBM_create(numDisparities=16*num, blockSize=blockSize)
disp = stereo.compute(gray_l, gray_r)
disp = cv2.normalize(disp, disp, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)

# Apply mean filtering to the disparity map
kernel_size = 40  # Kernel size can be adjusted
disp_filtered = cv2.blur(disp, (kernel_size, kernel_size))

# Recalculate 3D point cloud
threeD = cv2.reprojectImageTo3D(disp_filtered.astype(np.float32) / 16., camera_config.Q)

# Display depth for queried points
for (x, y) in query_points:
    if 0 <= x < threeD.shape[1] and 0 <= y < threeD.shape[0]:
        depth = threeD[y, x, 2]
        print(" {:.2f}".format(depth))
    else:
        print("Point ({}, {}) is out of bounds.".format(x, y))


