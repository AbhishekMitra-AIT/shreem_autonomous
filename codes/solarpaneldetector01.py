#!/usr/bin/python3

import cv2
import numpy as np
from picamera2 import Picamera2

def detect_solar_panels(frame):
    # Convert to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Apply Gaussian blur to reduce noise
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    
    # Apply adaptive thresholding
    thresh = cv2.adaptiveThreshold(
        blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, 
        cv2.THRESH_BINARY_INV, 11, 2
    )
    
    # Morphological operations to remove noise and enhance panel shapes
    kernel = np.ones((3,3), np.uint8)
    thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
    thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
    
    # Find contours
    contours, _ = cv2.findContours(
        thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    
    # Frame center coordinates
    frame_center_x = frame.shape[1] // 2
    frame_center_y = frame.shape[0] // 2
    
    panels = []
    for contour in contours:
        # Calculate contour area and perimeter
        area = cv2.contourArea(contour)
        perimeter = cv2.arcLength(contour, True)
        
        # Filter small contours and calculate approximated polygon
        if area > 1000:  # Adjust this threshold based on your needs
            approx = cv2.approxPolyDP(contour, 0.02 * perimeter, True)
            
            # Check if the shape is rectangular (4 vertices)
            if len(approx) == 4:
                # Calculate aspect ratio
                x, y, w, h = cv2.boundingRect(approx)
                aspect_ratio = float(w)/h
                
                # Solar panels typically have aspect ratio between 1.5 and 2.5
                if 1.5 <= aspect_ratio <= 2.5:
                    panels.append({
                        'contour': contour,
                        'approx': approx,
                        'center': (x + w//2, y + h//2),
                        'bounds': (x, y, w, h)
                    })
    
    return panels, (frame_center_x, frame_center_y)

def calculate_distances(panel, frame_center):
    # Calculate distances from panel center to frame center
    panel_center = panel['center']
    dx = panel_center[0] - frame_center[0]
    dy = panel_center[1] - frame_center[1]
    
    # Calculate Euclidean distance
    distance = np.sqrt(dx**2 + dy**2)
    
    return {
        'horizontal': dx,
        'vertical': dy,
        'euclidean': distance
    }

# Initialize the cameras
picam2_0 = Picamera2(camera_num=0)
picam2_0.configure(picam2_0.create_preview_configuration(
    main={"format": 'XRGB8888', "size": (640, 480)}
))
picam2_0.start()

picam2_1 = Picamera2(camera_num=1)
picam2_1.configure(picam2_1.create_preview_configuration(
    main={"format": 'XRGB8888', "size": (640, 480)}
))
picam2_1.start()

current_camera = 0

try:
    while True:
        # Capture image from the current camera
        if current_camera == 0:
            frame = picam2_0.capture_array()
        else:
            frame = picam2_1.capture_array()
            
        # Detect solar panels
        panels, frame_center = detect_solar_panels(frame)
        
        # Draw detected panels and calculate distances
        for panel in panels:
            # Draw contour
            cv2.drawContours(frame, [panel['contour']], -1, (0, 255, 0), 2)
            
            # Calculate distances
            distances = calculate_distances(panel, frame_center)
            
            # Draw center point of panel
            cv2.circle(frame, panel['center'], 5, (0, 0, 255), -1)
            
            # Draw line from frame center to panel center
            cv2.line(frame, frame_center, panel['center'], (255, 0, 0), 2)
            
            # Display distances
            x, y = panel['bounds'][:2]
            cv2.putText(frame, f"H: {distances['horizontal']:.1f}px", 
                       (x, y - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            cv2.putText(frame, f"V: {distances['vertical']:.1f}px", 
                       (x, y - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            cv2.putText(frame, f"D: {distances['euclidean']:.1f}px", 
                       (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        # Draw frame center
        cv2.circle(frame, frame_center, 5, (255, 0, 0), -1)
        
        # Display the frame
        cv2.imshow("Solar Panel Detection", frame)
        
        # Check for key press to toggle cameras or quit
        key = cv2.waitKey(1)
        if key & 0xFF == ord('q'):
            break
        elif key & 0xFF == ord('c'):
            current_camera = 1 - current_camera

finally:
    # Cleanup
    cv2.destroyAllWindows()
    picam2_0.stop()
    picam2_1.stop()