#!/usr/bin/python3

import cv2
from picamera2 import Picamera2

# Load the face and smile detectors
face_detector = cv2.CascadeClassifier("/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml")
smile_detector = cv2.CascadeClassifier("/usr/share/opencv4/haarcascades/haarcascade_smile.xml")

# Check if the classifiers are loaded correctly
if face_detector.empty() or smile_detector.empty():
    print("Error loading Haar cascade files.")
    exit(1)

# Initialize the cameras
picam2_0 = Picamera2(camera_num=0)
picam2_0.configure(picam2_0.create_preview_configuration(main={"format": 'XRGB8888', "size": (640, 480)}))
picam2_0.start()

picam2_1 = Picamera2(camera_num=1)
picam2_1.configure(picam2_1.create_preview_configuration(main={"format": 'XRGB8888', "size": (640, 480)}))
picam2_1.start()

# Variable to toggle between cameras
current_camera = 0

try:
    while True:
        # Capture image from the current camera
        if current_camera == 0:
            im = picam2_0.capture_array()
        else:
            im = picam2_1.capture_array()

        # Convert to grayscale for face and smile detection
        grey = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)

        # Detect faces
        faces = face_detector.detectMultiScale(grey, scaleFactor=1.3, minNeighbors=5)
        for (x, y, w, h) in faces:
            cv2.rectangle(im, (x, y), (x + w, y + h), (0, 255, 255), 2)
            print(f"Face detected in Camera {current_camera}.")

            # Region of interest for smile detection
            roi_gray = grey[y:y + h, x:x + w]
            roi_color = im[y:y + h, x:x + w]

            # Detect smiles
            smiles = smile_detector.detectMultiScale(roi_gray, scaleFactor=1.8, minNeighbors=20)
            for (sx, sy, sw, sh) in smiles:
                cv2.rectangle(im, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(im, "Thanks for Smiling..", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2, cv2.LINE_AA)
                print(f"Smile detected in Camera {current_camera}.")
                break

        # Display the image from the current camera
        cv2.imshow("Camera Feed", im)

        # Check for key press to toggle cameras
        key = cv2.waitKey(1)
        if key & 0xFF == ord('q'):
            break
        elif key & 0xFF == ord('c'):  # Press 'c' to switch cameras
            current_camera = 1 - current_camera  # Toggle between 0 and 1

finally:
    # Cleanup
    cv2.destroyAllWindows()
    picam2_0.stop()
    picam2_1.stop()
