# -*- coding: utf-8 -*-
import cv2
import numpy as np

# Create window for depth visualization
cv2.namedWindow("depth")
cv2.moveWindow('depth', 600, 0)

# Create trackbars for parameter adjustment
cv2.createTrackbar('num', 'depth', 3, 10, lambda x: None)
cv2.createTrackbar('blockSize', 'depth', 7, 255, lambda x: None)


left_camera_matrix = np.array([[554.25469119,   0.        , 320.5       ],
                               [  0.        , 554.25469119, 256.5       ],
                               [  0.        ,   0.        ,   1.        ]])

left_distortion = np.array([0.0, 0.0, 0.0, 0.0, 0.0])

right_camera_matrix = np.array([[554.25469119,   0.        , 320.5       ],
                               [  0.        , 554.25469119, 256.5       ],
                               [  0.        ,   0.        ,   1.        ]])

right_distortion = np.array([0.0, 0.0, 0.0, 0.0, 0.0])


R = np.array([[1.0, 0.0, 0.0],
             [0.0,  1.0, 0.0],
             [-0.0, 0.0, 1.0]])
T = np.array([0.320, 0.000, -0.000])

size = (640,512)

R1,R2,P1,P2,Q,roi1,roi2 = cv2.stereoRectify(left_camera_matrix,left_distortion,
                                            right_camera_matrix,right_distortion,
                                            size,R,T)

newcameramatrix1,roi1 = cv2.getOptimalNewCameraMatrix(left_camera_matrix,left_distortion,size,0,size)
newcameramatrix2,roi2 = cv2.getOptimalNewCameraMatrix(right_camera_matrix,right_distortion,size,0,size)

left_map1,left_map2 = cv2.initUndistortRectifyMap(left_camera_matrix,left_distortion,None,newcameramatrix1,size,cv2.CV_16SC2)
right_map1,right_map2 = cv2.initUndistortRectifyMap(right_camera_matrix,right_distortion,None,newcameramatrix2,size,cv2.CV_16SC2)

def callbackFunc(e, x, y, f, p):
    if e == cv2.EVENT_LBUTTONDOWN:
        print(threeD[y][x])

# Set mouse callback function
cv2.setMouseCallback("depth", callbackFunc, None)

imgL = np.zeros((480, 640, 3), np.uint8)
imgR = np.zeros((480, 640, 3), np.uint8)

points = np.array([[100, 100], [100, 100], [100, 100], [100, 100]])# query_point input is here !
center_point = np.mean(points, axis=0)
query_points = [(int(center_point[0]), int(center_point[1]))]


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
threeD = cv2.reprojectImageTo3D(disp_filtered.astype(np.float32) / 16., Q)

# Display depth for queried points
for (x, y) in query_points:
    if 0 <= x < threeD.shape[1] and 0 <= y < threeD.shape[0]:
        depth = threeD[y, x, 2]
        print(" {:.2f}".format(depth))
    else:
        print("Point ({}, {}) is out of bounds.".format(x, y))

cv2.imshow('depth', disp)
cv2.waitKey(100000)
cv2.destroyAllWindows()
