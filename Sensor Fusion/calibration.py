import cv2
import numpy as np
import glob
import os

# Define the chessboard size
chessboard_size = (10, 7)  # 10 inner corners in rows, 7 inner corners in columns

# Define the size of each square in meters
square_size = 0.05  # 50mm squares

# Termination criteria for corner refinement
criteria = (cv2.TermCriteria_EPS + cv2.TermCriteria_MAX_ITER, 30, 0.001)

# Prepare object points based on the chessboard size
objp = np.zeros((np.prod(chessboard_size), 3), np.float32)
objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)
objp = objp * square_size

# Arrays to store object points and image points from all the images
objpoints = []  # 3d points in real world space
imgpoints = []  # 2d points in image plane

# Directory containing the images
image_directory = "/home/nuc1/Videos"
image_pattern = os.path.join(image_directory, '*.png')

# Load images
images = glob.glob(image_pattern)

if len(images) == 0:
    print(f"No images found in {image_directory}. Please check the path and file format.")
else:
    for fname in images:
        print(f"Processing image: {fname}")
        
        # Load the image
        img = cv2.imread(fname)
        
        # Verify the image is loaded
        if img is None:
            print(f"Failed to load image: {fname}")
            continue
        
        # Convert to grayscale
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
        # Find the chessboard corners
        ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)
        
        if ret:
            objpoints.append(objp)
            
            # Refine corner positions
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            imgpoints.append(corners2)
            
            # Draw and display the corners
            cv2.drawChessboardCorners(img, chessboard_size, corners2, ret)
            cv2.imshow('Chessboard Corners', img)
            cv2.waitKey(500)
        else:
            print(f"Chessboard corners not found in image: {fname}")
    
    cv2.destroyAllWindows()

    # Perform camera calibration
    if len(objpoints) > 0 and len(imgpoints) > 0:
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
        
        # Save the calibration results
        calibration_data = {
            "camera_matrix": mtx,
            "dist_coeff": dist,
            "rotation_vectors": rvecs,
            "translation_vectors": tvecs
        }
        np.savez("camera_calibration_data.npz", **calibration_data)

        # Print the calibration results
        print("Camera calibration successful!")
        print(f"Camera Matrix:\n{mtx}")
        print(f"Distortion Coefficients:\n{dist}")
    else:
        print("Not enough valid images for calibration. Please ensure the chessboard is fully visible in multiple images.")

