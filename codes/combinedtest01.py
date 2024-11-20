#!/usr/bin/python3

import cv2
import numpy as np
from picamera2 import Picamera2
from roboflow import Roboflow
import time
from PIL import Image
import io
import os

class SolarPanelDetector:
    def __init__(self):
        # Initialize Roboflow
        rf = Roboflow(api_key="M3blcH9P3dyh4EtbWumD")
        self.model = rf.workspace().project("solarpaneldetection-edpsn").version(1).model
        
        # Initialize cameras
        self.picam2_0 = Picamera2(camera_num=0)
        self.picam2_0.configure(self.picam2_0.create_preview_configuration(
            main={"format": 'XRGB8888', "size": (640, 480)}
        ))
        self.picam2_0.start()

        self.picam2_1 = Picamera2(camera_num=1)
        self.picam2_1.configure(self.picam2_1.create_preview_configuration(
            main={"format": 'XRGB8888', "size": (640, 480)}
        ))
        self.picam2_1.start()

        self.current_camera = 0

    def convert_xrgb_to_bgr(self, frame):
        """Convert XRGB format to BGR"""
        if frame.shape[2] == 4:
            # XRGB to BGR conversion
            frame = frame[:, :, 1:]  # Remove the X channel
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        return frame

    def detect_solar_panels(self, frame):
        """Traditional CV-based detection"""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        thresh = cv2.adaptiveThreshold(
            blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, 
            cv2.THRESH_BINARY_INV, 11, 2
        )
        
        kernel = np.ones((3,3), np.uint8)
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
        
        contours, _ = cv2.findContours(
            thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        
        frame_center_x = frame.shape[1] // 2
        frame_center_y = frame.shape[0] // 2
        
        panels = []
        for contour in contours:
            area = cv2.contourArea(contour)
            perimeter = cv2.arcLength(contour, True)
            
            if area > 1000:
                approx = cv2.approxPolyDP(contour, 0.02 * perimeter, True)
                
                if len(approx) == 4:
                    x, y, w, h = cv2.boundingRect(approx)
                    aspect_ratio = float(w)/h
                    
                    if 1.5 <= aspect_ratio <= 2.5:
                        panels.append({
                            'contour': contour,
                            'approx': approx,
                            'center': (x + w//2, y + h//2),
                            'bounds': (x, y, w, h)
                        })
        
        return panels, (frame_center_x, frame_center_y)

    def apply_roboflow_segmentation(self, frame):
        """Apply Roboflow model for segmentation"""
        try:
            # Save frame to temporary file
            temp_filename = "temp_frame.jpg"
            cv2.imwrite(temp_filename, frame)
            
            # Get predictions from Roboflow model
            predictions = self.model.predict(temp_filename, confidence=40).json()
            
            # Create mask with same dimensions as input frame
            mask = np.zeros((frame.shape[0], frame.shape[1]), dtype=np.uint8)
            
            # Draw predictions on the mask
            for prediction in predictions.get('predictions', []):
                if 'points' in prediction:
                    points = prediction['points']
                    if points:
                        points = np.array([[int(p['x']), int(p['y'])] for p in points])
                        cv2.fillPoly(mask, [points], 255)
                elif all(k in prediction for k in ['x', 'y', 'width', 'height']):
                    x = int(prediction['x'])
                    y = int(prediction['y'])
                    width = int(prediction['width'])
                    height = int(prediction['height'])
                    
                    x1 = int(x - width/2)
                    y1 = int(y - height/2)
                    x2 = int(x + width/2)
                    y2 = int(y + height/2)
                    
                    cv2.rectangle(mask, (x1, y1), (x2, y2), 255, -1)
            
            # Clean up temporary file
            if os.path.exists(temp_filename):
                os.remove(temp_filename)
                
            return mask
            
        except Exception as e:
            print(f"Error in segmentation: {str(e)}")
            return np.zeros((frame.shape[0], frame.shape[1]), dtype=np.uint8)

    def calculate_distances(self, panel, frame_center):
        """Calculate distances from panel center to frame center"""
        panel_center = panel['center']
        dx = panel_center[0] - frame_center[0]
        dy = panel_center[1] - frame_center[1]
        distance = np.sqrt(dx**2 + dy**2)
        
        return {
            'horizontal': dx,
            'vertical': dy,
            'euclidean': distance
        }

    def create_visualization(self, frame, mask, panels, frame_center):
        """Create visualization with proper dimension handling"""
        # Ensure mask has same dimensions as frame
        if mask.shape[:2] != frame.shape[:2]:
            mask = cv2.resize(mask, (frame.shape[1], frame.shape[0]))
        
        # Convert grayscale mask to BGR for colored visualization
        colored_mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        colored_mask[mask > 0] = [0, 0, 255]  # Red overlay for detected regions
        
        # Create overlay
        overlay = cv2.addWeighted(frame, 0.7, colored_mask, 0.3, 0)
        
        # Draw panel information
        for panel in panels:
            # Draw contour
            cv2.drawContours(overlay, [panel['contour']], -1, (0, 255, 0), 2)
            
            # Draw center point and line
            cv2.circle(overlay, panel['center'], 5, (0, 0, 255), -1)
            cv2.line(overlay, frame_center, panel['center'], (255, 0, 0), 2)
            
            # Calculate and display distances
            distances = self.calculate_distances(panel, frame_center)
            x, y = panel['bounds'][:2]
            cv2.putText(overlay, f"H: {distances['horizontal']:.1f}px", 
                       (x, y - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            cv2.putText(overlay, f"V: {distances['vertical']:.1f}px", 
                       (x, y - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            cv2.putText(overlay, f"D: {distances['euclidean']:.1f}px", 
                       (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        # Draw frame center
        cv2.circle(overlay, frame_center, 5, (255, 0, 0), -1)
        
        return overlay

    def run(self):
        try:
            while True:
                # Capture frame
                if self.current_camera == 0:
                    frame = self.picam2_0.capture_array()
                else:
                    frame = self.picam2_1.capture_array()

                # Convert XRGB to BGR
                frame = self.convert_xrgb_to_bgr(frame)

                # Traditional detection
                panels, frame_center = self.detect_solar_panels(frame)
                
                # Roboflow segmentation
                segmentation_mask = self.apply_roboflow_segmentation(frame)
                
                # Create visualization
                visualization = self.create_visualization(frame, segmentation_mask, panels, frame_center)
                
                # Display the frame
                cv2.imshow("Solar Panel Detection with Segmentation", visualization)
                
                # Check for key press to toggle cameras or quit
                key = cv2.waitKey(1)
                if key & 0xFF == ord('q'):
                    break
                elif key & 0xFF == ord('c'):
                    self.current_camera = 1 - current_camera

        finally:
            # Cleanup
            cv2.destroyAllWindows()
            self.picam2_0.stop()
            self.picam2_1.stop()

if __name__ == "__main__":
    detector = SolarPanelDetector()
    detector.run()