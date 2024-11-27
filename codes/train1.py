# from ultralytics import YOLO

# # Load a pretrained YOLO11 segment model
# model = YOLO(""yolo11n-seg.pt"")

# model.info()

# results = model.predict(0, show=True, save=False)


from ultralytics import YOLO
import cv2

# Load your custom-trained YOLOv11 model
model = YOLO(r"D:\Aegeus\SelfDriving\runs\last.pt")  # Replace with the path to your trained weights

# Initialize the camera (use the appropriate camera index; 0 is usually the default webcam)
camera = cv2.VideoCapture(0)  # Replace 0 with the video source if needed

# Check if the camera is opened successfully
if not camera.isOpened():
    print("Error: Could not open camera.")
    exit()

print("Press 'q' to exit.")

while True:
    # Capture a frame from the camera
    ret, frame = camera.read()
    if not ret:
        print("Error: Could not read frame.")
        break

    # Perform instance segmentation on the frame
    results = model.predict(frame, show=False, save=False, imgsz=640)

    # Annotate the frame with predictions
    annotated_frame = results[0].plot()

    # Display the annotated frame
    cv2.imshow("Live Instance Segmentation", annotated_frame)

    # Exit the loop when 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and close all OpenCV windows
camera.release()
cv2.destroyAllWindows()
