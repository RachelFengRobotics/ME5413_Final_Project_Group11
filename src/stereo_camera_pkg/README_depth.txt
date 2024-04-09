## Getting Started

### Prerequisites

- Python 3.6 or higher
- OpenCV (cv2) with contributions package
- NumPy

Install the necessary packages using pip if you haven't done so already:

```sh
pip install numpy opencv-python opencv-contrib-python
```

### Running the Script

1. Ensure you have a pair of stereo images named `left.png` and `right.png` in the same directory as the script.
2. Run the script:

```sh
python depth_estimation.py
```

3. Adjust the `num` and `blockSize` parameters using the trackbars in the "depth" window to optimize the depth map.
4. Click on any point within the depth map window to print its depth value to the console.

## How It Works

The script performs the following steps:

1. Initializes stereo camera parameters and rectification maps.
2. Creates a GUI window with trackbars for parameter adjustment.
3. Reads the left and right images, applies pyramid downscaling for performance, and converts them to grayscale.
4. Computes the disparity map using either StereoBM or StereoSGBM (StereoBM is used in the final settings).
5. Applies mean filtering to the disparity map to smooth out noise.
6. Recalculates the 3D point cloud and allows depth querying through mouse clicks in the depth map.

## Notes

- The script is currently set up to work with images downscaled for performance. Adjust `pyrDown` calls as needed based on your image resolution and system capabilities.
- The camera parameters and distortion coefficients are hardcoded and should be adjusted to match your stereo camera setup.
- The script includes examples of both StereoBM and StereoSGBM methods for educational purposes. Adjust according to your needs.
