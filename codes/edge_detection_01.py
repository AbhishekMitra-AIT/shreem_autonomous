#!/usr/bin/python3

import cv2
from picamera2 import Picamera2

def initialize_camera(camera_num):
    """Initialize a camera with specified settings"""
    picam = Picamera2(camera_num=camera_num)
    picam.configure(picam.create_preview_configuration(main={"format": 'XRGB8888', "size": (640, 480)}))
    picam.start()
    return picam

def detect_edges(frame, low_threshold=50, high_threshold=150, kernel_size=5):
    """Perform edge detection on the input frame"""
    # Convert to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Apply Gaussian blur to reduce noise
    blurred = cv2.GaussianBlur(gray, (kernel_size, kernel_size), 0)
    
    # Apply Canny edge detection
    edges = cv2.Canny(blurred, low_threshold, high_threshold)
    
    # Convert back to BGR for display (optional)
    edges_colored = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
    
    return edges_colored

def main():
    # Initialize both cameras
    try:
        picam2_0 = initialize_camera(0)
        picam2_1 = initialize_camera(1)
        print("Both cameras initialized successfully")
    except Exception as e:
        print(f"Error initializing cameras: {str(e)}")
        return

    # Variable to toggle between cameras
    current_camera = 0
    
    # Edge detection parameters
    low_threshold = 50
    high_threshold = 150
    kernel_size = 5

    # Create window with trackbars
    cv2.namedWindow("Edge Detection")
    cv2.createTrackbar("Low Threshold", "Edge Detection", low_threshold, 255, lambda x: None)
    cv2.createTrackbar("High Threshold", "Edge Detection", high_threshold, 255, lambda x: None)

    try:
        while True:
            # Capture image from the current camera
            if current_camera == 0:
                im = picam2_0.capture_array()
            else:
                im = picam2_1.capture_array()

            # Get current trackbar values
            low_threshold = cv2.getTrackbarPos("Low Threshold", "Edge Detection")
            high_threshold = cv2.getTrackbarPos("High Threshold", "Edge Detection")

            # Perform edge detection
            edges = detect_edges(im, low_threshold, high_threshold, kernel_size)

            # Add camera indicator
            cv2.putText(edges, f"Camera {current_camera}", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            # Display the edge detection result
            cv2.imshow("Edge Detection", edges)

            # Check for key press
            key = cv2.waitKey(1) & 0xFF
            
            # 'q' to quit
            if key == ord('q'):
                break
            # 'c' to switch cameras
            elif key == ord('c'):
                current_camera = 1 - current_camera
                print(f"Switched to Camera {current_camera}")
            # 's' to save image
            elif key == ord('s'):
                cv2.imwrite(f'edge_detection_cam{current_camera}.jpg', edges)
                print(f"Saved edge detection image from Camera {current_camera}")

    except KeyboardInterrupt:
        print("\nProgram interrupted by user")
    except Exception as e:
        print(f"Error during execution: {str(e)}")
    finally:
        # Cleanup
        cv2.destroyAllWindows()
        picam2_0.stop()
        picam2_1.stop()
        print("Cameras stopped and windows closed")

if __name__ == "__main__":
    main()