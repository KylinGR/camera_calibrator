# ROS Camera Calibration

This ROS package provides a camera calibration program that utilizes ROS image topics to perform camera calibration.

## Features

- Subscribes to the ROS image topic `/camera/image_raw` to capture image data.
- Detects the corners of a checkerboard pattern in the captured images.
- Calculates the camera intrinsic and distortion parameters using the OpenCV camera calibration algorithm.
- Outputs the calibration results in a YAML file.
- Supports automatic camera calibration without user interaction.

## Prerequisites

- ROS (Tested on ROS Melodic and ROS Noetic) 
- OpenCV (Included in most ROS distributions)
- Python 3.x

## Usage

1. Clone the repository to your ROS workspace:

cd ~/catkin_ws

[git clone https://github.com/your-username/ros-camera-calibration.git](https://github.com/KylinGR/camera_calibrator.git)


2. Build the ROS package:

catkin_make --source camera_calibrator/  --build build/camera_calibrator/

3. Source the ROS environment:

source ~/catkin_ws/devel/setup.bash

4. Run the camera calibration node:

roslaunch camera_calibrator camera_calibrator.launch


This will start the calibration process and subscribe to the `/camera/image_raw` topic.

5. Place a checkerboard pattern in the camera's field of view. The program will automatically detect the corners and perform the calibration.

6. Once the calibration is complete, the results will be saved to a YAML file named `camera_calibration.yaml` in the current working directory.


## Troubleshooting

- Ensure that the ROS topic `/camera/image_raw` is being published correctly and that the camera is properly connected.
- Check the ROS logs for any error messages or debugging information.
- Verify that the checkerboard pattern is visible and properly illuminated in the camera's field of view.

## Contributing

If you find any issues or have suggestions for improvements, please feel free to open a new issue or submit a pull request.