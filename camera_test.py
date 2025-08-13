import cv2
import numpy as np

# Load the saved camera calibration data (from the camera_calibration.npz file)
calibration_data = np.load('camera_calibration.npz')
mtx = calibration_data['mtx']
dist = calibration_data['dist']

# Convert pixel coordinates to real-world coordinates (in millimeters)
def pixel_to_world(pixel_coords):
    # Inverse of camera matrix
    inv_mtx = np.linalg.inv(mtx)

    # Distort the point back to real-world coordinates
    x, y = pixel_coords
    normalized_coords = np.array([[x, y, 1]], dtype=np.float32).T  # Homogeneous coordinates
    world_coords = inv_mtx @ normalized_coords  # Applying inverse of camera matrix
    return world_coords[0][0], world_coords[1][0]

# Mouse callback function to mark the point and convert to world coordinates
def mouse_callback(event, x, y, flags, param):
    global point, show_coords

    if event == cv2.EVENT_LBUTTONDOWN:  # If left mouse button is clicked
        point = (x, y)

        # Convert pixel coordinates to world coordinates (in mm)
        world_x, world_y = pixel_to_world((x, y))
        show_coords = f"Pixel (u, v): ({x}, {y}) -> X: {world_x:.2f} mm, Y: {world_y:.2f} mm"
        
        print(f"Clicked at Pixel ({x}, {y}) -> World Coordinates: ({world_x:.2f}, {world_y:.2f})")

# Initialize variables
point = None  # To store the last clicked point
show_coords = ""  # To display the coordinates
cap = cv2.VideoCapture(0)  # Open the camera

if not cap.isOpened():
    print("Error: Could not open camera.")
    exit()

cv2.namedWindow("Live Feed")
cv2.setMouseCallback("Live Feed", mouse_callback)

while True:
    ret, frame = cap.read()

    if not ret:
        print("Failed to capture frame.")
        break

    # If a point is clicked, mark it in green
    if point:
        cv2.circle(frame, point, 5, (0, 255, 0), -1)  # Draw a green circle at the clicked point
        cv2.putText(frame, show_coords, (point[0] + 10, point[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

    # Show the live video feed
    cv2.imshow("Live Feed", frame)

    key = cv2.waitKey(1) & 0xFF

    # Clear the point when SPACE is pressed
    if key == 32:  # Spacebar to clear the point
        point = None
        show_coords = ""

    # Exit the program when ESC is pressed
    if key == 27:  # ESC key to quit
        break

# Release resources and close windows
cap.release()
cv2.destroyAllWindows()

