import cv2
import numpy as np
import glob

def calibrate_camera(chessboard_size=(9, 6), square_size=1.0):
    """
    Calibrate the camera using images of a chessboard pattern.

    :param chessboard_size: Tuple (width, height) - number of inner corners in the chessboard.
    :param square_size: The physical size of a square on the chessboard (in some unit, e.g., mm).
    """
    # Prepare 3D points for the chessboard pattern in real world space (3D coordinates)
    obj_points = []  # 3D points in real world space
    img_points = []  # 2D points in image plane

    # Define the chessboard pattern: (0,0,0), (1,0,0), (2,0,0), ..., (width-1, height-1, 0)
    objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
    objp[:, :2] = np.indices(chessboard_size).T.reshape(-1, 2)
    objp *= square_size  # Scale the points by the square size

    # Load images of the chessboard
    images = glob.glob("image*.jpg")
    print(f"Found {len(images)} calibration images.")
    
    for image_path in images:
        # Read each image
        img = cv2.imread(image_path)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Find chessboard corners
        ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)

        if ret:
            # Add object points and image points
            obj_points.append(objp)
            img_points.append(corners)

            # Draw and display the corners (optional)
            cv2.drawChessboardCorners(img, chessboard_size, corners, ret)
            cv2.imshow("Chessboard", img)
            cv2.waitKey(500)  # Show for 500ms

    cv2.destroyAllWindows()

    if len(obj_points) > 0:
        # Calibrate the camera
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, gray.shape[::-1], None, None)
        
        if ret:
            print("Camera calibrated successfully!")
            print("Camera matrix (intrinsic parameters):")
            print(mtx)
            print("Distortion coefficients:")
            print(dist)
            
            # Save calibration results
            np.savez("camera_calibration.npz", mtx=mtx, dist=dist, rvecs=rvecs, tvecs=tvecs)

        else:
            print("Camera calibration failed.")
    else:
        print("Not enough valid images for calibration.")

# Start the calibration process
calibrate_camera()

