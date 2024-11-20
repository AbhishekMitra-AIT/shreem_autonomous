#!/usr/bin/python3

import cv2
import numpy as np
from picamera2 import Picamera2

class RectangleEdgeDetector:
    def __init__(self):
        # Initialize the cameras
        self.picam2_0 = Picamera2(camera_num=0)
        self.picam2_0.configure(self.picam2_0.create_preview_configuration(
            main={"format": 'XRGB8888', "size": (640, 480)}))
        self.picam2_0.start()

        self.picam2_1 = Picamera2(camera_num=1)
        self.picam2_1.configure(self.picam2_1.create_preview_configuration(
            main={"format": 'XRGB8888', "size": (640, 480)}))
        self.picam2_1.start()

        # Current camera selection
        self.current_camera = 0

        # Edge detection parameters
        self.canny_low = 50
        self.canny_high = 150
        self.min_area = 1000  # Minimum area for rectangle detection
        self.epsilon_factor = 0.02  # Polygon approximation parameter

    def detect_rectangle_edges(self, frame):
        """Detect rectangular edges in the frame"""
        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Apply Gaussian blur to reduce noise
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        
        # Edge detection using Canny
        edges = cv2.Canny(blurred, self.canny_low, self.canny_high)
        
        # Find contours
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Create blank image for drawing edges
        edge_image = np.zeros_like(frame)
        
        # Process each contour
        for contour in contours:
            # Filter small contours
            if cv2.contourArea(contour) < self.min_area:
                continue
                
            # Approximate the contour to a polygon
            epsilon = self.epsilon_factor * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)
            
            # Check if the polygon has 4 vertices (rectangular)
            if len(approx) == 4:
                # Draw the rectangle edges in white
                cv2.drawContours(edge_image, [approx], -1, (255, 255, 255), 2)
        
        return edge_image

    def run(self):
        """Main processing loop"""
        try:
            while True:
                # Capture frame from current camera
                if self.current_camera == 0:
                    frame = self.picam2_0.capture_array()
                else:
                    frame = self.picam2_1.capture_array()

                # Detect and draw rectangle edges
                edge_image = self.detect_rectangle_edges(frame)

                # Display the edge detection result
                cv2.imshow("Rectangle Edge Detection", edge_image)

                # Handle key presses
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                elif key == ord('c'):  # Switch cameras
                    self.current_camera = 1 - self.current_camera
                elif key == ord('+'):  # Increase sensitivity
                    self.canny_low = max(0, self.canny_low - 10)
                    self.canny_high = max(0, self.canny_high - 10)
                elif key == ord('-'):  # Decrease sensitivity
                    self.canny_low = min(255, self.canny_low + 10)
                    self.canny_high = min(255, self.canny_high + 10)
                elif key == ord('a'):  # Adjust minimum area
                    self.min_area = max(100, self.min_area - 100)
                elif key == ord('s'):  # Adjust minimum area
                    self.min_area = min(5000, self.min_area + 100)

        except Exception as e:
            print(f"Error: {str(e)}")
        finally:
            self.cleanup()

    def cleanup(self):
        """Cleanup resources"""
        cv2.destroyAllWindows()
        self.picam2_0.stop()
        self.picam2_1.stop()

if __name__ == "__main__":
    print("Starting Rectangle Edge Detection...")
    print("Controls:")
    print("  'q' - Quit")
    print("  'c' - Switch cameras")
    print("  '+' - Increase edge detection sensitivity")
    print("  '-' - Decrease edge detection sensitivity")
    print("  'a' - Decrease minimum area threshold")
    print("  's' - Increase minimum area threshold")
    
    detector = RectangleEdgeDetector()
    detector.run()