from picamera2 import Picamera2
import cv2
import os
import time

# Create the folder if it doesn't already exist.
folder_name = "calib_images"
if not os.path.exists(folder_name):
    os.makedirs(folder_name)

# Initialize Raspberry Pi Camera
picam2 = Picamera2()
preview_config = picam2.create_preview_configuration(
    main={"size": (640, 480), "format": "RGB888"}
)
picam2.configure(preview_config)
picam2.start()

time.sleep(1)  # Give the camera a moment to warm up

count = 0

print("Press 'space bar' to capture an image, or 'q' to quit program.")

while True:
    frame = picam2.capture_array()

    cv2.imshow("Capture Calibration Images", frame)

    key = cv2.waitKey(1) & 0xFF

    if key == ord(" "):
        img_name = os.path.join(folder_name, f"img_{count}.jpg")
        cv2.imwrite(img_name, frame)
        print(f"Saved: {img_name}")
        count += 1

    elif key == ord("q"):
        break

picam2.stop()
cv2.destroyAllWindows()
