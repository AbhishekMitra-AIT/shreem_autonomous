#!/usr/bin/python3

import cv2
import numpy as np
from picamera2 import Picamera2
from roboflow import Roboflow
import os
import time

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
        self.last_print_time = 0
        print("\n=== Starting Live Segmentation with Distance Measurement ===")
        print("Press 'q' to quit, 'c' to switch cameras")
        print("Distance measurements will update every 0.5 seconds\n")

    def convert_xrgb_to_bgr(self, frame):
        if frame.shape[2] == 4:
            frame = frame[:, :, 1:]
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        return frame

    def calculate_distances(self, prediction, frame_center):
        if 'points' in prediction:
            points = prediction['points']
            if points:
                points_arr = np.array([[int(p['x']), int(p['y'])] for p in points])
                centroid = np.mean(points_arr, axis=0).astype(int)
        else:
            x = int(prediction['x'])
            y = int(prediction['y'])
            centroid = np.array([x, y])

        dx = centroid[0] - frame_center[0]
        dy = centroid[1] - frame_center[1]
        euclidean_distance = np.sqrt(dx**2 + dy**2)

        return {
            'centroid': tuple(centroid),
            'horizontal': dx,
            'vertical': dy,
            'euclidean': euclidean_distance
        }

    def print_distances(self, distances, prediction):
        current_time = time.time()
        if current_time - self.last_print_time >= 0.5:
            print("\033[2J\033[H")  # Clear console
            print(f"=== Detection Information ===")
            print(f"Camera: {self.current_camera}")
            print(f"Raw Prediction Data:")
            for key, value in prediction.items():
                print(f"  {key}: {value}")
            print(f"\nDistance Measurements:")
            print(f"Horizontal Distance: {distances['horizontal']:>8.1f} px {'(Right)' if distances['horizontal'] > 0 else '(Left) '}")
            print(f"Vertical Distance:   {distances['vertical']:>8.1f} px {'(Down) ' if distances['vertical'] > 0 else '(Up)   '}")
            print(f"Euclidean Distance:  {distances['euclidean']:>8.1f} px")
            print("===========================\n")
            self.last_print_time = current_time

    def apply_segmentation(self, frame):
        try:
            temp_filename = "temp_frame.jpg"
            cv2.imwrite(temp_filename, frame)
            
            # Get predictions from Roboflow model
            result = self.model.predict(temp_filename, confidence=40).json()
            predictions = result.get('predictions', [])
            
            # Debug print for predictions
            if predictions:
                print(f"\nReceived {len(predictions)} predictions")
                print("Classes detected:", [p.get('class', 'unknown') for p in predictions])
            else:
                print("\nNo predictions received")
            
            # Create mask
            mask = np.zeros((frame.shape[0], frame.shape[1]), dtype=np.uint8)
            
            for prediction in predictions:
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
            
            if os.path.exists(temp_filename):
                os.remove(temp_filename)
                
            return mask, predictions
            
        except Exception as e:
            print(f"\nError in segmentation: {str(e)}")
            return np.zeros((frame.shape[0], frame.shape[1]), dtype=np.uint8), []

    def create_visualization(self, frame, mask, predictions):
        frame_center = (frame.shape[1] // 2, frame.shape[0] // 2)
        
        if mask.shape[:2] != frame.shape[:2]:
            mask = cv2.resize(mask, (frame.shape[1], frame.shape[0]))
        
        colored_mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        colored_mask[mask > 0] = [0, 0, 255]
        
        overlay = cv2.addWeighted(frame, 0.7, colored_mask, 0.3, 0)
        
        cv2.circle(overlay, frame_center, 5, (255, 0, 0), -1)
        cv2.putText(overlay, "Frame Center", (frame_center[0] + 10, frame_center[1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
        
        # Process all predictions regardless of class
        for prediction in predictions:
            distances = self.calculate_distances(prediction, frame_center)
            centroid = distances['centroid']
            
            # Print distances to console for all detections
            self.print_distances(distances, prediction)
            
            # Use different colors for different classes
            color = (0, 255, 0)  # Default green
            if prediction.get('class') != 'solarpaneledge':
                color = (255, 165, 0)  # Orange for other classes
            
            cv2.circle(overlay, centroid, 5, color, -1)
            cv2.line(overlay, frame_center, centroid, color, 2)
            
            text_x = centroid[0] + 10
            text_y = centroid[1]
            cv2.putText(overlay, f"Class: {prediction.get('class', 'unknown')}", 
                       (text_x, text_y - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            cv2.putText(overlay, f"H: {distances['horizontal']:.1f}px", 
                       (text_x, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            cv2.putText(overlay, f"V: {distances['vertical']:.1f}px", 
                       (text_x, text_y + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            cv2.putText(overlay, f"D: {distances['euclidean']:.1f}px", 
                       (text_x, text_y + 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        
        return overlay

    def run(self):
        try:
            while True:
                if self.current_camera == 0:
                    frame = self.picam2_0.capture_array()
                else:
                    frame = self.picam2_1.capture_array()

                frame = self.convert_xrgb_to_bgr(frame)
                segmentation_mask, predictions = self.apply_segmentation(frame)
                visualization = self.create_visualization(frame, segmentation_mask, predictions)
                cv2.imshow("Live Segmentation with Distance Measurement", visualization)
                
                key = cv2.waitKey(1)
                if key & 0xFF == ord('q'):
                    break
                elif key & 0xFF == ord('c'):
                    self.current_camera = 1 - self.current_camera
                    print(f"\nSwitched to Camera {self.current_camera}")

        finally:
            cv2.destroyAllWindows()
            self.picam2_0.stop()
            self.picam2_1.stop()

if __name__ == "__main__":
    segmentation = LiveSegmentation()
    segmentation.run()