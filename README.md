# Autonomous-Robot-Line-Tracking
Vision-based autonomous robot lane tracking using ROS 2 and Pure Pursuit algorithms with single and multiple lookahead strategies.

Autonomous Robot Lane Tracking with ROS 2 

This project implements and evaluates two vision-based lane tracking algorithms for an autonomous mobile robot using ROS 2 and real-time image processing. The goal is to enable reliable, reactive navigation without GPS or pre-built maps by processing a live camera feed to detect and follow a marked lane. 

Overview 

Two variations of the Pure Pursuit path-following algorithm were developed and tested: 

Single Lookahead Point 

Scans one horizontal strip near the bottom of the image. 

Calculates lane offset and curvature to adjust wheel speeds. 

Includes a hard turn mode for sharp 90° turns. 

Prioritizes precision but operates at a low base speed, leading to rigid motion and oscillations on straight sections. 

Multiple Lookahead Points (Three Lookaheads) 

Samples three rows at different heights in the image. 

Uses a weighted average to anticipate curves for smoother motion. 

Adds go-straight override and bias adjustment for improved curve handling and noise tolerance. 

Achieves higher speed, stability, and adaptability compared to the single lookahead approach. 

Technical Details 

Hardware: Raspberry Pi with camera, Arduino Mega motor controller. 

Image Processing: RGB to grayscale conversion, adaptive thresholding, centroid detection of lane pixels. 

Control: Differential drive kinematics, curvature-based wheel speed commands via serial communication. 

Languages & Tools: C++ (ROS 2 nodes), OpenCV for camera calibration, SSH-based remote development on Raspberry Pi. 

Results 

Both algorithms successfully completed the evaluation course. 

Single lookahead was accurate but slow and less adaptive. 

Multiple lookahead produced smoother, faster, and more robust navigation, particularly around curves and cross sections. 

Key Learnings 

Small parameter changes (lookahead distance, scanline weights) can drastically affect real-world performance. 

Environmental conditions (e.g., lighting) must be considered in vision-based navigation. 

Multi-lookahead approaches provide significant benefits in responsiveness and stability. 
