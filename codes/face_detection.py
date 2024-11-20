#!/usr/bin/python3

import cv2
from picamera2 import Picamera2
import datetime

# Load the face and smile detectors
face_detector = cv2.CascadeClassifier("/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml")
smile_detector = cv2.CascadeClassifier("/usr/share/opencv4/haarcascades/haarcascade_smile.xml")

# Check if the classifiers are loaded correctly
if face_detector.empty() or smile_detector.empty():
    print("Error loading Haar cascade files.")
    exit(1)

cv2.startWindowThread()

# Initialize the camera
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"format": 'XRGB8888', "size": (640, 480)}))
picam2.start()

while True:
    im = picam2.capture_array()

    # Convert to grayscale
    grey = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)

    # Detect faces
    faces = face_detector.detectMultiScale(grey, scaleFactor=1.3, minNeighbors=5)

    for (x, y, w, h) in faces:
        # Draw a rectangle around the face
        cv2.rectangle(im, (x, y), (x + w, y + h), (0, 255, 0), 2)
        print("Face detected.")

        # Region of interest for the smile detector
        face_region = grey[y:y + h, x:x + w]

        # Detect smiles within the face region
        smiles = smile_detector.detectMultiScale(face_region, scaleFactor=1.7, minNeighbors=15)

        if len(smiles) > 0:
            # Draw a rectangle around the smile
            for (sx, sy, sw, sh) in smiles:
                cv2.rectangle(im, (x + sx, y + sy), (x + sx + sw, y + sy + sh), (255, 0, 0), 2)
                print("Smile detected.")

            # Take a photo when a smile is detected
            timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            photo_path = f"smile_{timestamp}.jpg"
            cv2.imwrite(photo_path, im)
            print(f"Photo saved as {photo_path}")

            # Break to avoid capturing multiple photos per smile detection
            break

    # Display the image
    cv2.imshow("Camera", im)

    # Exit if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Cleanup
cv2.destroyAllWindows()
picam2.stop()
