#!/usr/bin/python3

import cv2
import numpy as np
from picamera2 import Picamera2

class EdgeDetector:
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

        # Detection parameters
        self.white_threshold = 200  # Threshold for white detection (0-255)
        self.min_area = 1000  # Minimum contour area
        self.kernel = np.ones((5,5), np.uint8)
        
        # Glare reduction parameters
        self.glare_threshold = 240  # Threshold for identifying glare
        self.saturation_threshold = 30  # Threshold for saturation in glare detection
        self.local_adapt_block_size = 51  # Must be odd number
        self.local_adapt_c = 10
        self.bilateral_d = 9
        self.bilateral_sigma_color = 75
        self.bilateral_sigma_space = 75

    def reduce_glare(self, frame):
        """Apply glare reduction techniques to the frame"""
        # Ensure frame is in correct format (BGR)
        if len(frame.shape) == 2:  # If grayscale
            frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        elif frame.shape[2] == 4:  # If RGBA
            frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
            
        # Create a copy for bilateral filter
        frame_copy = frame.copy()
        
        # Convert to HSV color space
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv)
        
        # Identify potential glare areas (high value, low saturation)
        glare_mask = np.logical_and(
            v > self.glare_threshold,
            s < self.saturation_threshold
        ).astype(np.uint8) * 255
        
        # Apply bilateral filter to reduce glare while preserving edges
        bilateral = cv2.bilateralFilter(
            frame_copy,
            self.bilateral_d,
            self.bilateral_sigma_color,
            self.bilateral_sigma_space
        )
        
        # Create local adaptive threshold to handle varying lighting
        gray_bilateral = cv2.cvtColor(bilateral, cv2.COLOR_BGR2GRAY)
        local_adaptive = cv2.adaptiveThreshold(
            gray_bilateral,
            255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY,
            self.local_adapt_block_size,
            self.local_adapt_c
        )
        
        # Combine original frame and bilateral filtered frame based on glare mask
        result = frame.copy()
        result[glare_mask > 0] = bilateral[glare_mask > 0]
        
        return result, glare_mask, local_adaptive

    def preprocess_frame(self, frame):
        """Preprocess the frame to isolate white lines while handling glare"""
        # Handle XRGB8888 format
        if frame.shape[2] == 4:
            frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
            
        # Apply glare reduction
        glare_reduced, glare_mask, local_adaptive = self.reduce_glare(frame)
        
        # Convert to grayscale
        gray = cv2.cvtColor(glare_reduced, cv2.COLOR_BGR2GRAY)
        
        # Apply Gaussian blur to reduce noise
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        
        # Combine local adaptive threshold with regular threshold
        _, white_mask = cv2.threshold(
            blurred, 
            self.white_threshold, 
            255, 
            cv2.THRESH_BINARY
        )
        
        # Combine thresholds while excluding glare areas
        combined_mask = cv2.bitwise_and(
            white_mask,
            cv2.bitwise_not(glare_mask)
        )
        
        # Use local adaptive threshold in glare areas
        combined_mask[glare_mask > 0] = local_adaptive[glare_mask > 0]
        
        # Morphological operations to clean up the mask
        dilated = cv2.dilate(combined_mask, self.kernel, iterations=1)
        white_regions = cv2.erode(dilated, self.kernel, iterations=1)
        
        return white_regions, glare_mask

    def detect_edges(self, frame):
        """Detect white edges and calculate distances from center"""
        # Get frame dimensions
        height, width = frame.shape[:2]
        center_x = width // 2
        center_y = height // 2
        
        # Get white regions and glare mask
        white_regions, glare_mask = self.preprocess_frame(frame)
        
        # Find contours in the white regions
        contours, _ = cv2.findContours(
            white_regions, 
            cv2.RETR_EXTERNAL, 
            cv2.CHAIN_APPROX_SIMPLE
        )
        
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
                    # Check if contour is mostly in non-glare area
                    mask = np.zeros_like(glare_mask)
                    cv2.drawContours(mask, [approx], -1, 255, -1)
                    glare_overlap = cv2.bitwise_and(mask, glare_mask)
                    glare_percentage = np.sum(glare_overlap > 0) / np.sum(mask > 0)
                    
                    # Only include contour if less than 30% overlaps with glare
                    if glare_percentage < 0.3:
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
        
        return panel_contours, distances, white_regions, glare_mask

    def draw_overlay(self, frame, contours, distances, glare_mask):
        """Draw detection overlay on the frame"""
        # Ensure frame is in correct format
        if frame.shape[2] == 4:
            frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
            
        height, width = frame.shape[:2]
        center_x = width // 2
        center_y = height // 2
        
        # Create overlay for glare visualization
        glare_overlay = frame.copy()
        glare_overlay[glare_mask > 0] = [0, 0, 255]  # Red tint for glare areas
        
        # Blend with original frame
        frame = cv2.addWeighted(frame, 0.7, glare_overlay, 0.3, 0)
        
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

        return frame

    def run(self):
        """Main processing loop"""
        try:
            while True:
                # Capture frame from current camera
                if self.current_camera == 0:
                    frame = self.picam2_0.capture_array()
                else:
                    frame = self.picam2_1.capture_array()

                # Detect white regions and calculate distances
                contours, distances, white_regions, glare_mask = self.detect_edges(frame)
                
                # Create visualization frames
                white_frame = cv2.cvtColor(white_regions, cv2.COLOR_GRAY2BGR)
                overlay_frame = self.draw_overlay(frame.copy(), contours, distances, glare_mask)
                
                # Display the results
                cv2.imshow("White Regions", white_frame)
                cv2.imshow("Panel Detection", overlay_frame)
                cv2.imshow("Glare Mask", glare_mask)

                # Handle key presses
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                elif key == ord('c'):
                    self.current_camera = 1 - self.current_camera
                elif key == ord('+'):
                    self.white_threshold = min(255, self.white_threshold + 5)
                    print(f"White threshold: {self.white_threshold}")
                elif key == ord('-'):
                    self.white_threshold = max(0, self.white_threshold - 5)
                    print(f"White threshold: {self.white_threshold}")
                elif key == ord('g'):
                    self.glare_threshold = min(255, self.glare_threshold + 5)
                    print(f"Glare threshold: {self.glare_threshold}")
                elif key == ord('h'):
                    self.glare_threshold = max(0, self.glare_threshold - 5)
                    print(f"Glare threshold: {self.glare_threshold}")
                    
        finally:
            cv2.destroyAllWindows()
            self.picam2_0.stop()
            self.picam2_1.stop()

if __name__ == "__main__":
    detector = EdgeDetector()
    detector.run()