import cv2
import time

def capture_images(camera_index=0):
    # Open the camera
    cap = cv2.VideoCapture(camera_index, cv2.CAP_V4L2)
    
    if not cap.isOpened():
        print("Error: Unable to open camera.")
        return
    
    # Set camera properties if needed (optional)
    #cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)  # Set frame width
    #cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)  # Set frame height
    cap.set(cv2.CAP_PROP_FPS, 30)  # Set FPS
    
    image_count = 1  # Initialize image count
    print(f"Press SPACE to capture an image. Press ESC to quit.")
    
    while True:
        # Capture a frame
        ret, frame = cap.read()
        
        if not ret:
            print("Error: Failed to capture frame.")
            break
        
        # Show the current frame in a window
        cv2.imshow("Camera Feed", frame)

        # Wait for a key press
        key = cv2.waitKey(1) & 0xFF

        if key == 27:  # ESC key to quit
            print("Exiting...")
            break
        elif key == 32:  # SPACEBAR key to capture the image
            image_filename = f"image{image_count}.jpg"
            cv2.imwrite(image_filename, frame)
            print(f"Captured and saved {image_filename}")
            image_count += 1

    # Release the camera and close any open windows
    cap.release()
    cv2.destroyAllWindows()

# Start capturing images
capture_images()

