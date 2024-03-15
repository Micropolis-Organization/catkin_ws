# Lane Detection for Mobile Robots

This is a ROS node implementation for lane detection and motion planning for mobile robots. The node uses a deep learning model (YOLOPv2) for lane detection and publishes a velocity command for the robot to follow the lane.

## Methodology

To combine motion planning for a mobile robot with lane detection, we follow these steps:

 - Implement a lane detection algorithm to extract lane boundaries from the camera images.
 - Use the extracted lane boundaries to estimate the robot's position relative to the lane.
 - Based on the estimated position, plan a safe and feasible trajectory for the robot to follow.
 - Continuously repeat the above steps and update the planned trajectory to avoid obstacles and stay within the lane.
 - Implement the planned trajectory on the robot by controlling its actuators (e.g. wheel speed, steering angle).

## Prerequisites

- ROS Kinetic or later
- Python 2.7 or 3.x
- Tensorflow 2.x
- OpenCV

## Installation

1. Clone the repository to your workspace

$ git clone https://github.com/your-username/lane-detection-ros.git

2. Install required packages

$ pip install -r requirements.txt


3. Build the package

$ catkin_make


## Usage

1. Launch the ROS node

$ roslaunch lane_detection_ros lane_detection_node.launch


2. Start publishing images to the `/image_raw` topic. The node will detect the lanes in the images and publish a velocity command to follow the lane.

## Output

The node publishes a velocity command to the `/cmd_vel` topic. The linear velocity of the robot is set based on the error between the center of the lane and the center of the image. The angular velocity is set to steer the robot towards the center of the lane.

