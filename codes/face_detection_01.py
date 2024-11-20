#!/usr/bin/python3

import cv2
from picamera2 import Picamera2
import datetime
import os

# Load the face and smile detectors
face_detector = cv2.CascadeClassifier("/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml")
smile_detector = cv2.CascadeClassifier("/usr/share/opencv4/haarcascades/haarcascade_smile.xml")

# Check if the classifiers are loaded correctly
if face_detector.empty() or smile_detector.empty():
    print("Error loading Haar cascade files.")
    exit(1)

# Initialize the camera
picam2 = Picamera2(camera_num=1)
picam2.configure(picam2.create_preview_configuration(main={"format": 'XRGB8888', "size": (640, 480)}))
picam2.start()

# Create a directory to save captured photos
if not os.path.exists("captured_smiles"):
    os.makedirs("captured_smiles")

try:
    while True:
        im = picam2.capture_array()
        
        # Convert to grayscale
        grey = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
        
        # Detect faces
        faces = face_detector.detectMultiScale(grey, scaleFactor=1.3, minNeighbors=5)
        
        for (x, y, w, h) in faces:
            # Draw a yellow rectangle around the face
            cv2.rectangle(im, (x, y), (x + w, y + h), (0, 255, 255), 2)
            print("Face detected.")
            
            # Region of interest for smile detection
            roi_gray = grey[y:y+h, x:x+w]
            roi_color = im[y:y+h, x:x+w]
            
            # Detect smiles
            smiles = smile_detector.detectMultiScale(roi_gray, scaleFactor=1.8, minNeighbors=20)
            
            for (sx, sy, sw, sh) in smiles:
                # Change face rectangle to green when smile is detected
                cv2.rectangle(im, (x, y), (x + w, y + h), (0, 255, 0), 2)

                # Display "Smile Detected" message
                cv2.putText(im, "Thanks for Smiling..", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2, cv2.LINE_AA)
                print("Smile detected.")
                
                # Take a photo when a smile is detected
                timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
                photo_path = f"captured_smiles/smile_{timestamp}.jpg"
                cv2.imwrite(photo_path, im)
                print(f"Photo saved as {photo_path}")
                
                # Break to avoid capturing multiple photos per smile detection
                break
        
        # Display the image
        cv2.imshow("Camera", im)
        
        # Exit if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    # Cleanup
    cv2.destroyAllWindows()
    picam2.stop()
