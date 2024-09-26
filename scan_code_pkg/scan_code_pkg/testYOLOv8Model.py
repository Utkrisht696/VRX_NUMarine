from ultralytics import YOLO
import cv2
import torch

print(torch.cuda.is_available()) 

# Load the YOLOv8 model
model = YOLO('scanDockDeliver.pt')  # Replace 'yolov8n.pt' with the path to your YOLOv8 model if different
model.to('cuda')  # Ensure the model uses GPU
# Load the image
image_path = "images/VRX1.png"  # Replace with the path to your image
image = cv2.imread(image_path)

# Perform inference
results = model.predict(source=image, device='cuda', save=False, conf=0.5)  # 'save=True' saves the results

# Display the results on the image
annotated_image = results[0].plot()  # This draws the bounding boxes and labels on the image

# Create a window that allows for resizing
cv2.namedWindow('Annotated Image', cv2.WINDOW_NORMAL) 

# Show the image with bounding boxes
cv2.imshow('Annotated Image', annotated_image)
cv2.waitKey(0)
cv2.destroyAllWindows()

