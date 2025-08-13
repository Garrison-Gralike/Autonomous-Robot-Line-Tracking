import cv2
import numpy as np

# Load the saved calibration data (from the camera_calibration.npz file)
calibration_data = np.load('camera_calibration.npz')
mtx = calibration_data['mtx']
dist = calibration_data['dist']

# Load an image to undistort
img = cv2.imread('image10.jpg')  # Replace with your image path

# Undistort the image using the camera matrix and distortion coefficients
undistorted_img = cv2.undistort(img, mtx, dist)

# Show the original (distorted) and undistorted images in separate windows
cv2.imshow('Distorted Image', img)
cv2.imshow('Undistorted Image', undistorted_img)

# Wait for a key press and close all windows
cv2.waitKey(0)
cv2.destroyAllWindows()

