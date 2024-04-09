import cv2
import numpy as np

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

