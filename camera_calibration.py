import cv2
import numpy as np
import glob
import os

CHECKERBOARD = (9, 6) # Define the dimensions of the board 
SQUARE_SIZE_MM = 20.0 # Define the real-world size of a single square in mm

def calibrate_camera(images_folder):
    """
    Reads checkerboard images, detects corners, and calculates the camera matrix.
    Saves the resulting matrix and distortion coefficients to a file.
    """
    # Criteria for the subpixel corner refinment algorithm
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    # Arrays to store object points and image points
    object_points = []
    image_points = []

    # Prepare the theoretical 3D points of the checkerboard
    objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
    objp *= SQUARE_SIZE_MM

    # Fetch all jpg images from the specified folder
    images = glob.glob(os.path.join(images_folder, '*.jpg'))

    if not images:
        print(f"No images found in {images_folder}")
        return
    
    print(f"Found {len(images)} images. starting calibration..")

    gray_shape = None

    for fname in images:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        gray_shape = gray.shape[::-1]

        # Find the checkerboard corners
        ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)

        if ret:
            object_points.append(objp)
            # Refine the corner locations for better accuracy
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            image_points.append(corners2)
            
            # Draw and display the corners to verify detection
            cv2.drawChessboardCorners(img, CHECKERBOARD, corners2, ret)
            cv2.imshow('Calibration - Press any key for next or wait', img)
            cv2.waitKey(1000)

    cv2.destroyAllWindows()

    if len(object_points) > 0:
        print("Calculating camera matrix..")
        # Preform the actual camera calibration math
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(object_points, image_points, gray_shape, None, None)
        # Save the calibration data to a compressed file fo future use in main script
        np.savez("calib_data.npz", mtx=mtx, dist=dist)
        print("Calibration successful. Data saved to 'calib_data.npz'.")
    else:
        print("Falied to find checkerboard corners in the images.")
        
if __name__ == "__main__":
    # Target folder contining the checkerboard images
    calibrate_camera('calib_images')
