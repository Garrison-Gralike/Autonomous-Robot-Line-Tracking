# Autonomous-Robot-Line-Tracking
## Vision-based autonomous robot lane tracking using ROS 2 and Pure Pursuit algorithms with single and multiple lookahead strategies.

## About The Project
This project implements and evaluates two vision-based lane tracking algorithms for an autonomous mobile robot using ROS 2 and real-time image processing. The goal is to enable reliable, reactive navigation without GPS or pre-built maps by processing a live camera feed to detect and follow a marked lane. Two variations of the Pure Pursuit path-following algorithm were developed and tested: 


### Built With

* [![C++](https://img.shields.io/badge/C++-00599C?style=for-the-badge&logo=cplusplus&logoColor=white)][C++-url]
* [![ROS 2](https://img.shields.io/badge/ROS%202-22314E?style=for-the-badge&logo=ros&logoColor=white)][ROS2-url]
* [![OpenCV](https://img.shields.io/badge/OpenCV-27338e?style=for-the-badge&logo=opencv&logoColor=white)][OpenCV-url]
* [![Raspberry Pi](https://img.shields.io/badge/Raspberry%20Pi-A22846?style=for-the-badge&logo=raspberrypi&logoColor=white)][RaspberryPi-url]
* [![Arduino](https://img.shields.io/badge/Arduino-00979D?style=for-the-badge&logo=arduino&logoColor=white)][Arduino-url]
* [![SSH](https://img.shields.io/badge/SSH-000000?style=for-the-badge&logo=gnubash&logoColor=white)][SSH-url]

[C++-url]: https://isocpp.org/
[ROS2-url]: https://docs.ros.org/en/humble/
[OpenCV-url]: https://opencv.org/
[RaspberryPi-url]: https://www.raspberrypi.com/
[Arduino-url]: https://www.arduino.cc/
[SSH-url]: https://www.ssh.com/academy/ssh





## Getting Started
### Prerequisites

You will need the following installed to build and run this project:

* Ubuntu 22.04 LTS with ROS 2 Humble
* GCC or Clang with C++17 support
* colcon build system
* OpenCV (with development headers)
* Git

```sh
# Example: Install ROS 2 Humble desktop version
sudo apt update && sudo apt install ros-humble-desktop
```

```sh
# Install build tools
sudo apt install build-essential cmake git python3-colcon-common-extensions
```
```sh
# Install OpenCV
sudo apt install libopencv-dev python3-opencv
```

### Installation
1. Clone the repository to your ROS2 workspace
   ```sh
   cd ~/ros2_ws/src
   git clone https://github.com/Garrison-Gralike/Autonomus-Robot_Line_Tracking.git
   ```
2. Build the Project
   ```sh
   cd ~/ros2_ws
   colcon build --packages-select image_sub
   ```
3. Source the Workspace
   ```sh
   source install/setup.bash
   ```
4. Set Serial Permissions for Rasberry Pi to Audrino Link
   ```sh
   sudo usermod -a -G dialout $USER
   # log out and back in after running this
   ```
5. Run the Node
   ```sh
   ros2 run image_sub image_sub
   ```
## Usage 

### Demo

[![Watch the video](https://img.youtube.com/vi/3xNNGDgtfRI/hqdefault.jpg)](https://youtube.com/shorts/3xNNGDgtfRI?si=BMPnr4udRJfGMk4d)

[Watch on YouTube](https://youtube.com/shorts/3xNNGDgtfRI?si=BMPnr4udRJfGMk4d)






### Single Lookahead Point 

- Scans one horizontal strip near the bottom of the image. 
- Calculates lane offset and curvature to adjust wheel speeds. 
- Includes a hard turn mode for sharp 90° turns. 
- Prioritizes precision but operates at a low base speed, leading to rigid motion and oscillations on straight sections. 


### Multiple Lookahead Points (Three Lookaheads) 

- Samples three rows at different heights in the image. 
- Uses a weighted average to anticipate curves for smoother motion. 
- Adds go-straight override and bias adjustment for improved curve handling and noise tolerance. 
- Achieves higher speed, stability, and adaptability compared to the single lookahead approach. 


### Technical Details 

- Hardware: Raspberry Pi with camera, Arduino Mega motor controller. 
- Image Processing: RGB to grayscale conversion, adaptive thresholding, centroid detection of lane pixels. 
- Control: Differential drive kinematics, curvature-based wheel speed commands via serial communication. 
- Languages & Tools: C++ (ROS 2 nodes), OpenCV for camera calibration, SSH-based remote development on Raspberry Pi. 


### Results

Both algorithms successfully completed the test course. The single lookahead was accurate but slow and less adaptive. 
Multiple lookahead produced smoother, faster, and more robust navigation, without compromising accuracy. Performed particularly well around curves and cross sections


### Key Tips 

Small parameter changes (lookahead distance, scanline weights) can drastically affect performance. 
Environmental conditions (e.g., lighting) must be considered in vision-based navigation.
Multi-lookahead approaches provide significant benefits in responsiveness and stability.



## Contributors

## Contact
- [LinkedIn](https://www.linkedin.com/in/<garrison-gralike-56164b253>) – <ggralike1@gmail.com>

