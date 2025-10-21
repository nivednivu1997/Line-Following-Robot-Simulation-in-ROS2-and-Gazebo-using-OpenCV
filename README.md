# Line Following Robot Simulation in ROS2 and Gazebo using OpenCV

This project implements a **line-following robot** in ROS2 using a PID controller and OpenCV for computer vision. The robot can detect a line in a simulated environment (Gazebo or real camera feed) and follow it autonomously.

---

## Features

- Uses **OpenCV** to process camera images and detect the line.
- Implements a **PID controller** for smooth line following.
- Automatically **searches for the line** if it is lost.
- Compatible with **ROS2 (Humble)**.
- Can be used with simulated robots in **Gazebo** or a real robot with a camera.

---

## Dependencies

- ROS2 (tested on Humble)
- OpenCV (`cv2`)
- `cv_bridge`
- Python 3
- `numpy`

Install dependencies (example for ROS2 Humble):

```bash
sudo apt update
sudo apt install ros-humble-cv-bridge ros-humble-sensor-msgs ros-humble-geometry-msgs python3-opencv python3-numpy
```
---

### Usage

## 1) Clone the repository:

```bash
git clone https://github.com/yourusername/ros2-gazebo-linefollower-opencv.git
cd ros2-gazebo-linefollower-opencv
```
## 2) Build the package:
```bash
cd line_follower_ws &&
colcon build &&
source install/setup.bash
```
## 3) Run the Line Follower Gazebo Environment:
```bash
ros2 launch nav2_simple_commander route_example_launch.py x_pose:=1.2 y_pose:=0.5 yaw:=1.57
```
## 4) Run Line Follower Python Node
```bash
cd line_follower_ws && python3 line_follower_node.py 

```

---

### How it Works
## Camera Feed Processing
- Captures the image from the robot's camera.

- Crops the lower 40% of the image to focus on the line.

- Converts the image to grayscale and applies binary thresholding.

- Cleans the image using morphological operations (erosion and dilation).

## Line Detection
- Computes the centroid of the line using OpenCV image moments.

- Calculates the error between the desired center (slightly shifted 5% to the right) and the detected line.

## PID Control
- Calculates control using Proportional (P), Integral (I), and Derivative (D) terms.

- Outputs the angular velocity to steer the robot.

- Maintains a constant forward speed while adjusting the steering.

## Fallback
If the line is not detected, the robot spins slowly to search for it.

### Parameters to Tune
- kp, ki, kd → PID gains for tuning the steering response.

- linear_speed → Forward speed of the robot.

- desired_center → Horizontal position of the line the robot should follow (0.5 = center, 0.55 = 5% offset to right).

- crop_top → Portion of image to focus on (default 0.6 → bottom 40%).

### File Structure
```bash
ros2-gazebo-linefollower-opencv/
├── line_follower_pid/
│   ├── line_follower_pid.py     # Main node implementing PID line following
│   ├── package.xml              # ROS2 package metadata
│   └── setup.py                 # Python package setup
├── README.md
```



License
This project is licensed under the MIT License.

Author
Your Name – Autonomous Mobile Robotics Developer
