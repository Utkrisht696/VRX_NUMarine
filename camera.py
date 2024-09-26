import PySpin

# Initialize the system instance
system = PySpin.System.GetInstance()

# Get the list of cameras
cam_list = system.GetCameras()
cam = cam_list[0]

# Initialize and configure the camera
cam.Init()

# Set acquisition mode to continuous
cam.AcquisitionMode.SetValue(PySpin.AcquisitionMode_Continuous)

# Begin acquiring images
cam.BeginAcquisition()

while True:
    try:
        # Retrieve the next image
        image_result = cam.GetNextImage()

        # Ensure the image is complete
        if image_result.IsIncomplete():
            print('Image incomplete with image status %d...' % image_result.GetImageStatus())

        else:
            # Convert the image to a numpy array
            image_data = image_result.GetNDArray()

            # Process your image here (e.g., display, save, etc.)

            # Display the image using OpenCV
            cv2.imshow('FLIR Camera', image_data)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        # Release the image
        image_result.Release()

    except PySpin.SpinnakerException as ex:
        print("Error: %s" % ex)

# End acquisition
cam.EndAcquisition()

# Deinitialize the camera
cam.DeInit()

# Release the camera
del cam

# Release the system instance
system.ReleaseInstance()

cv2.destroyAllWindows()



# import cv2
# import numpy as np
# import glob

# # Define the dimensions of the checkerboard
# CHECKERBOARD = (7, 6)
# SQUARE_SIZE = 25  # Size of a square in your defined units (e.g., millimeters)

# # Termination criteria for the corner sub-pixel refinement
# criteria = (cv2.TermCriteria_EPS + cv2.TermCriteria_MAX_ITER, 30, 0.001)

# # Prepare object points based on the checkerboard size and square size
# objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
# objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
# objp *= SQUARE_SIZE

# # Arrays to store object points and image points from all images
# objpoints = []  # 3d points in real-world space
# imgpoints = []  # 2d points in image plane

# # Load all images in the folder (change the path accordingly)
# images = glob.glob('calibration_images/*.jpg')

# gray = None  # Initialize gray to avoid the NameError

# for fname in images:
#     img = cv2.imread(fname)
#     gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

#     # Find the chessboard corners
#     ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)

#     if ret:
#         objpoints.append(objp)

#         # Refine the corner positions
#         corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
#         imgpoints.append(corners2)

#         # Draw and display the corners
#         cv2.drawChessboardCorners(img, CHECKERBOARD, corners2, ret)
#         cv2.imshow('img', img)
#         cv2.waitKey(500)

# cv2.destroyAllWindows()

# if gray is not None and len(objpoints) > 0 and len(imgpoints) > 0:
#     # Calibration
#     ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

#     # Save the camera matrix and distortion coefficients for later use
#     np.savez('calibration_data.npz', mtx=mtx, dist=dist, rvecs=rvecs, tvecs=tvecs)

#     # Print the camera matrix and distortion coefficients
#     print("Camera matrix:\n", mtx)
#     print("Distortion coefficients:\n", dist)

#     # Optional: Undistort a test image
#     img = cv2.imread('calibration_images/test_image.jpg')
#     h, w = img.shape[:2]
#     newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))

#     # Undistort
#     dst = cv2.undistort(img, mtx, dist, None, newcameramtx)

#     # Crop the image based on the ROI
#     x, y, w, h = roi
#     dst = dst[y:y + h, x:x + w]
#     cv2.imwrite('calibration_images/undistorted_image.jpg', dst)
# else:
#     print("No valid images found for calibration.")
