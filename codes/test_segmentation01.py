from roboflow import Roboflow

rf = Roboflow(api_key="M3blcH9P3dyh4EtbWumD")
project = rf.workspace().project("solarpaneldetection-edpsn")
model = project.version(1).model

# infer on a local image
print(model.predict("panel_cam0_20241115_170805.jpg").json())

# infer on an image hosted elsewhere
# print(model.predict("URL_OF_YOUR_IMAGE").json())

# save an image annotated with your predictions
model.predict("panel_cam0_20241115_170805.jpg").save("prediction.jpg")