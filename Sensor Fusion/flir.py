"""
========================================================================================================================================
Script Name:            debugBlackfly.py
Description:            This script uses the PySpin API to acquire frames from the Spinnaker Blackfly camera. 
                        There is a list of parameters in the configure_camera function that can be used to adjust the frame rate and picture quality.
                        Running this script will set those parameters. 
                                                
Author:                 Luke Thompson 
Created Date:           04/08/2024
Last Modified By:       Luke Thompson
Last Modified Date:     04/08/2024
Version:                1.0
Dependencies:           | OpenCV | PySpin |
Usage:                  Requires the PySpin API from https://www.flir.com/support-center/iis/machine-vision/downloads/spinnaker-sdk-download/spinnaker-sdk--download-files/
                        The Spinnaker SDK must be installed prior to installing PySpin (pretty sure anyways). 
                        Use the appropriate version for your CPU architecture: Spinnaker 4.0.0.116 for Ubuntu 22.04 Python (December 21, 2023)
License:                
========================================================================================================================================
"""

import PySpin
import cv2
import time
import numpy as np

def configure_camera(cam):
    cam.AcquisitionMode.SetValue(PySpin.AcquisitionMode_Continuous)     # Set camera to continuous mode
    cam.GainAuto.SetValue(PySpin.GainAuto_Off)                          # Turn off auto gain
    cam.ExposureAuto.SetValue(PySpin.ExposureAuto_Off)                  # Turn off auto exposure
    cam.BalanceWhiteAuto.SetValue(PySpin.BalanceWhiteAuto_Off)          # Turn off white balance
    cam.PixelFormat.SetValue(PySpin.PixelFormat_BayerRG8)               # Set pixel format to BayerRG8
    cam.ExposureTime.SetValue(20000)                                    # Set the exposure time in microseconds
    cam.Gain.SetValue(3)                                                # Set the gain value
    cam.BlackLevel.SetValue(2)                                          # Set brightness value
    cam.Gamma.SetValue(0.5)                                             # Set gamma value


def acquire_images(cam):
    try:
        cam.BeginAcquisition()
        
        prev_time = time.time()
        frame_count = 0

        blue =  (255, 0, 0) # BGR Format
        green = (0, 255, 0) # BGR Format
        red =   (0, 0, 255) # BGR Format

        while True:
            # Retrieve the next image
            image_result = cam.GetNextImage()
            
            # Ensure the image is valid
            if image_result.IsIncomplete():
                print(f"Image incomplete with image status {image_result.GetImageStatus()} ...")
                continue

            # Convert the image to numpy array
            image_data = image_result.GetNDArray()

            # Display image resolution
            height, width = image_data.shape[:2]

            # Get image format
            pixel_format = image_result.GetPixelFormatName()

             # Convert Bayer RGB to regular RGB
            image_data = cv2.cvtColor(image_data, cv2.COLOR_BayerRG2RGB)
            pixel_format = 'RGB'  # Update format label to RGB after conversion

            #image_data = enhance_image(image_data)

            # Calculate FPSq
            frame_count += 1
            current_time = time.time()
            fps = frame_count / (current_time - prev_time)

            # Display FPS, resolution, and format on the image
            display_image = image_data.copy()
            cv2.putText(display_image, f"Format: {pixel_format}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, red, 2)
            cv2.putText(display_image, f"Resolution: {width}x{height}", (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, red, 2)
            cv2.putText(display_image, f"FPS: {fps:.2f}", (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 1, red, 2)

            # Auto-fit the window size to the image size
            cv2.namedWindow("Image", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("Image", 1366, 768)
            cv2.imshow("Image", display_image)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            # Release the image
            image_result.Release()

    finally:
        # End acquisition
        cam.EndAcquisition()

def enhance_image(image):
    # Apply gamma correction
    gamma = 1.2  # Adjust as needed
    lookUpTable = np.empty((1, 256), np.uint8)
    for i in range(256):
        lookUpTable[0, i] = np.clip(pow(i / 255.0, gamma) * 255.0, 0, 255)
    image = cv2.LUT(image, lookUpTable)

    # Increase contrast and brightness
    alpha = 1.3  # Contrast control (1.0-3.0)
    beta = 40    # Brightness control (0-100)
    image = cv2.convertScaleAbs(image, alpha=alpha, beta=beta)

    # Convert to HSV to adjust saturation
    hsv_image = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
    hsv_image[..., 1] = cv2.add(hsv_image[..., 1], 30)  # Increase saturation
    image = cv2.cvtColor(hsv_image, cv2.COLOR_HSV2RGB)

    return image

def main():
    # Initialize the PySpin system
    system = PySpin.System.GetInstance()

    # Retrieve the list of cameras from the system
    cam_list = system.GetCameras()

    if cam_list.GetSize() == 0:
        print("No cameras detected.")
        return

    # Get the first camera from the list
    cam = cam_list.GetByIndex(0)

    try:
        # Initialize camera
        cam.Init()

        # Configure the camera
        configure_camera(cam)

        # Acquire and display images
        acquire_images(cam)

    finally:
        # Deinitialize the camera
        cam.DeInit()

        # Release the camera
        del cam

        # Release system resources
        cam_list.Clear()
        system.ReleaseInstance()

    # Close OpenCV windows
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()