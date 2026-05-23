import cv2
import os

# Create the folder if doesnt already exist.
folder_name = 'calib_images'
if not os.path.exists(folder_name):
    os.makedirs(folder_name)

cap = cv2.VideoCapture(0)
count = 0

print("Press 'space bar' to capture an image, or 'q' to quit program.")

while True:
    ret, frame = cap.read()
    cv2.imshow('Capture Calibration Images', frame)

    key = cv2.waitKey(1) & 0xFF
    if key == ord(' '):
        img_name = os.path.join(folder_name, f"img_{count}.jpg")
        cv2.imwrite(img_name, frame)
        print(f"Saved: {img_name}")
        count += 1
    elif key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
