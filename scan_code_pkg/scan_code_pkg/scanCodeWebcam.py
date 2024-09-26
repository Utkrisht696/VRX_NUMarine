import cv2
import numpy as np
from ultralytics import YOLO
import logging

# Configure logging to silence YOLO's output
logging.getLogger("ultralytics").setLevel(logging.ERROR)
logging.getLogger("pytorch_lightning").setLevel(logging.ERROR)

def get_color_mask(hsv_roi, color):
    if color == 'red':
        lower_bound = np.array([0, 100, 100])
        upper_bound = np.array([10, 255, 255])
        lower_bound2 = np.array([170, 100, 100])
        upper_bound2 = np.array([180, 255, 255])
        mask1 = cv2.inRange(hsv_roi, lower_bound, upper_bound)
        mask2 = cv2.inRange(hsv_roi, lower_bound2, upper_bound2)
        mask = mask1 | mask2
    elif color == 'green':
        lower_bound = np.array([36, 50, 50])
        upper_bound = np.array([89, 255, 255])
        mask = cv2.inRange(hsv_roi, lower_bound, upper_bound)
    elif color == 'blue':
        lower_bound = np.array([94, 80, 50])
        upper_bound = np.array([126, 255, 255])
        mask = cv2.inRange(hsv_roi, lower_bound, upper_bound)
    else:
        mask = None
    return mask

def get_predominant_color(roi):
    hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    
    red_mask = get_color_mask(hsv_roi, 'red')
    green_mask = get_color_mask(hsv_roi, 'green')
    blue_mask = get_color_mask(hsv_roi, 'blue')
    
    red_count = cv2.countNonZero(red_mask)
    green_count = cv2.countNonZero(green_mask)
    blue_count = cv2.countNonZero(blue_mask)
    
    pixel_threshold = 1000  # Adjust this value as needed
    
    if red_count > green_count and red_count > blue_count and red_count >= pixel_threshold:
        return 'Red', red_count
    elif green_count > red_count and green_count > blue_count and green_count >= pixel_threshold:
        return 'Green', green_count
    elif blue_count > red_count and blue_count > green_count and blue_count >= pixel_threshold:
        return 'Blue', blue_count
    else:
        return 'None', 0

# Load the YOLOv8 model
model = YOLO('detectMatrix25e.pt', verbose=False)  # Replace with the path to your YOLOv8 model if different

# Initialize video capture
cap = cv2.VideoCapture(0)  # Use 0 for webcam or replace with your camera index

# Initialize last detected color
last_detected_color = None

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Perform YOLO detection
    results = model(frame)

    current_frame_color = None

    for result in results:
        boxes = result.boxes.xyxy.cpu().numpy()
        for box in boxes:
            x1, y1, x2, y2 = map(int, box)
            
            # Extract ROI
            roi = frame[y1:y2, x1:x2]
            
            # Get predominant color in ROI
            color, pixel_count = get_predominant_color(roi)
            
            # Update current frame color
            if color != 'None':
                current_frame_color = color
            
            # Draw bounding box
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            
            # Add color label to image
            label = f"{color}: {pixel_count}"
            cv2.putText(frame, label, (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0,255,0), 2)

    # Check if color has changed
    if current_frame_color is not None and current_frame_color != last_detected_color:
        #print(f"Predominant color in bounding box ({x1}, {y1}, {x2}, {y2}): {color} ({pixel_count} pixels)")
        print(f"Color Detected: {current_frame_color}")
        last_detected_color = current_frame_color

    # Display the resulting frame
    cv2.imshow('YOLO Detection with Color Analysis', frame)
    
    # Break the loop on 'q' key press
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release everything when done
cap.release()
cv2.destroyAllWindows()