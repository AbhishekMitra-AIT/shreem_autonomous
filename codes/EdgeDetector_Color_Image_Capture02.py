#!/usr/bin/python3

import cv2
import numpy as np
from picamera2 import Picamera2
from datetime import datetime
import os

class EdgeDetector:
    def __init__(self):
        # Create output directory if it doesn't exist
        self.output_dir = "captured_images"
        os.makedirs(self.output_dir, exist_ok=True)

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
        self.min_area = 1000  # Minimum contour area to be considered a panel
        self.kernel = np.ones((5,5), np.uint8)

    def save_image(self, overlay_frame):
        """Save the panel detection overlay image"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        camera_id = f"cam{self.current_camera}"
        
        # Save panel detection image with overlay
        filename = os.path.join(
            self.output_dir, f"panel_{camera_id}_{timestamp}.jpg")
        cv2.imwrite(filename, overlay_frame)
        
        print(f"Image saved: {filename}")

    def preprocess_frame(self, frame):
        """Preprocess the frame for edge detection"""
        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Apply Gaussian blur to reduce noise
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        
        # Apply adaptive thresholding to handle varying lighting
        thresh = cv2.adaptiveThreshold(
            blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, 
            cv2.THRESH_BINARY, 11, 2)
        
        # Morphological operations to reduce noise
        dilated = cv2.dilate(thresh, self.kernel, iterations=1)
        eroded = cv2.erode(dilated, self.kernel, iterations=1)
        
        return eroded

    def detect_edges(self, frame):
        """Detect edges and calculate distances from center"""
        # Get frame dimensions
        height, width = frame.shape[:2]
        center_x = width // 2
        center_y = height // 2
        
        # Preprocess the frame
        processed = self.preprocess_frame(frame)
        
        # Apply Canny edge detection
        edges = cv2.Canny(processed, self.canny_low, self.canny_high)
        
        # Find contours
        contours, _ = cv2.findContours(
            edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Filter contours by area and approximate to polygons
        panel_contours = []
        distances = []
        
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > self.min_area:
                # Approximate the contour to a polygon
                epsilon = 0.02 * cv2.arcLength(contour, True)
                approx = cv2.approxPolyDP(contour, epsilon, True)
                
                # Check if the polygon has 4 sides (likely a panel)
                if len(approx) >= 4 and len(approx) <= 6:
                    panel_contours.append(approx)
                    
                    # Calculate center of contour
                    M = cv2.moments(approx)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        
                        # Calculate distance from frame center
                        distance = np.sqrt(
                            (cx - center_x)**2 + (cy - center_y)**2)
                        distances.append((distance, (cx, cy)))
        
        return panel_contours, distances, edges

    def draw_overlay(self, frame, contours, distances):
        """Draw detection overlay on the frame"""
        height, width = frame.shape[:2]
        center_x = width // 2
        center_y = height // 2
        
        # Draw frame center crosshair
        cv2.line(frame, (center_x - 20, center_y), 
                (center_x + 20, center_y), (0, 255, 0), 2)
        cv2.line(frame, (center_x, center_y - 20), 
                (center_x, center_y + 20), (0, 255, 0), 2)
        
        # Draw detected panels and distances
        for contour, (distance, center) in zip(contours, distances):
            # Draw panel contour
            cv2.drawContours(frame, [contour], -1, (0, 255, 255), 2)
            
            # Draw line from center to panel
            cv2.line(frame, (center_x, center_y), center, (255, 0, 0), 2)
            
            # Display distance
            cv2.putText(frame, f"{distance:.1f}px", 
                       (center[0] - 40, center[1] - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

    def run(self):
        """Main processing loop"""
        try:
            while True:
                # Capture frame from current camera
                if self.current_camera == 0:
                    frame = self.picam2_0.capture_array()
                else:
                    frame = self.picam2_1.capture_array()

                # Detect edges and calculate distances
                contours, distances, edges = self.detect_edges(frame)
                
                # Create visualization frames
                edge_frame = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
                overlay_frame = frame.copy()
                
                # Draw detection overlay
                self.draw_overlay(overlay_frame, contours, distances)
                
                # Display the results
                cv2.imshow("Edge Detection", edge_frame)
                cv2.imshow("Panel Detection", overlay_frame)

                # Handle key presses
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                elif key == ord('c'):
                    self.current_camera = 1 - self.current_camera
                elif key == ord('+'):
                    self.canny_high += 10
                elif key == ord('-'):
                    self.canny_high -= 10
                # Save color image with overlay on any other key press
                elif key != 255:  # 255 means no key was pressed
                    self.save_image(overlay_frame)
                    
        finally:
            cv2.destroyAllWindows()
            self.picam2_0.stop()
            self.picam2_1.stop()

if __name__ == "__main__":
    detector = EdgeDetector()
    detector.run()