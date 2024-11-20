#!/usr/bin/python3

import cv2
import numpy as np
from picamera2 import Picamera2
from roboflow import Roboflow
import os

class LiveSegmentation:
    def __init__(self):
        # Initialize Roboflow
        rf = Roboflow(api_key="M3blcH9P3dyh4EtbWumD")
        self.model = rf.workspace().project("solarpaneldetection-edpsn").version(3).model
        
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
            frame = frame[:, :, 1:]  # Remove the X channel
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        return frame

    def apply_segmentation(self, frame):
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

    def create_visualization(self, frame, mask):
        """Create visualization with segmentation overlay"""
        # Ensure mask has same dimensions as frame
        if mask.shape[:2] != frame.shape[:2]:
            mask = cv2.resize(mask, (frame.shape[1], frame.shape[0]))
        
        # Convert grayscale mask to BGR for colored visualization
        colored_mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        colored_mask[mask > 0] = [0, 0, 255]  # Red overlay for detected regions
        
        # Create overlay
        overlay = cv2.addWeighted(frame, 0.7, colored_mask, 0.3, 0)
        
        return overlay

    def run(self):
        try:
            while True:
                # Capture frame from current camera
                if self.current_camera == 0:
                    frame = self.picam2_0.capture_array()
                else:
                    frame = self.picam2_1.capture_array()

                # Convert XRGB to BGR
                frame = self.convert_xrgb_to_bgr(frame)
                
                # Apply segmentation
                segmentation_mask = self.apply_segmentation(frame)
                
                # Create visualization
                visualization = self.create_visualization(frame, segmentation_mask)
                
                # Display the frame
                cv2.imshow("Live Segmentation", visualization)
                
                # Check for key press to toggle cameras or quit
                key = cv2.waitKey(1)
                if key & 0xFF == ord('q'):
                    break
                elif key & 0xFF == ord('c'):
                    self.current_camera = 1 - self.current_camera

        finally:
            # Cleanup
            cv2.destroyAllWindows()
            self.picam2_0.stop()
            self.picam2_1.stop()

if __name__ == "__main__":
    segmentation = LiveSegmentation()
    segmentation.run()