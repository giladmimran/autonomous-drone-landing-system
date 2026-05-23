import cv2
import numpy as np
import time 
import os
import csv
from collections import deque
from drone_control import DroneTelemetry

# --- Configuration Constants ---
MARKER_SIZE_CM = 17.3           # Physical size of the ArUco marker in cm
LANDING_ALT_THRESHOLD = 30.0    # Altitude in cm below which final LAND mode is triggered
PRECISION_ALT_THRESHOLD = 300.0 # Altitude in cm below which precision approach is triggered
CENTER_THRESHOLD = 0.1          # Acceptable normalized error (10%) to consider the drone centered
FAILSAFE_TIMEOUT = 1.0          # Time in seconds without marker detection before triggering failsafe
CONFIDENCE_THRESHOLD = 90.0     # Minimum confidence % for marker detection to be considered valid
LOG_FILE = 'flight_log.csv'     # Target file for telemetry data logging

# --- 3D Model Definition ---
# Define the 3D corners of the ArUco marker in its own coordinate system (Z=0 plane)
# The corners are defined in the order: top-left, top-right, bottom-right, bottom-left
half_size = MARKER_SIZE_CM / 2.0
OBJ_POINTS = np.array([
    [-half_size,  half_size, 0],  # Top-left corner
    [ half_size,  half_size, 0],  # Top-right corner
    [ half_size, -half_size, 0],  # Bottom-right corner
    [-half_size, -half_size, 0]   # Bottom-left corner
], dtype=np.float32)

def initialize_log():
    """
    Initializes the CSV log file with column headers.
    This ensures we have structured data for post-flight analysis and engineering reports.
    """
    with open(LOG_FILE, mode='w', newline='') as f:
        csv.writer(f).writerow(['Timestamp', 'State', 'Real_Alt_cm', 'Est_Alt_cm', 'Err_X', 'Err_Y', 'Confidence'])

def log_data(state, real_alt, est_alt, err_x, err_y, confidence):
    """
    Appends real-time flight metrics to the CSV file at every frame.
    Records the system's state, actual telemetry altitude, vision-estimated altitude, and positioning errors.
    This data is crucial for analyzing the performance of the landing system, identifying trends, and supporting engineering decisions in the final report. 
    """
    with open(LOG_FILE, mode='a', newline='') as f:
        csv.writer(f).writerow([time.time(), state, real_alt, est_alt, err_x, err_y, confidence])

def draw_text(img, text, position, color=(255, 255, 255)):
    """
    Draws text with a thick black shadow on the video frame.
    This guarantees readability of the HUD (Heads-Up Display) against any background brightness.
    """
    cv2.putText(img, text, position, cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 3) # Black shadow
    cv2.putText(img, text, position, cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)     # White text

def main():
    """
    The main execution loop for the autonomous landing system.
    Integrates vision processing, background telemetry tracking, data logging, and state management.
        1. Initializes the background telemetry thread to continuously receive altitude data from the drone without blocking the vision processing.
        2. Sets up the camera and ArUco marker detection system, including loading camera calibration data if available for accurate distance estimation.
        3. Enters the main loop where it captures video frames, detects markers, estimates position and altitude, manages the landing state machine, sends commands to the drone, and logs all relevant data for analysis.
        4. Provides a visual HUD overlay on the video feed to display real-time telemetry and system status, enhancing situational awareness during testing and demonstration.
    """
    # Connects to the flight controller and continuously updates altitude without blocking the camera
    drone = DroneTelemetry('/dev/ttyAMA0', 921600)
    try:
        drone.connect()
        drone.start()
    except Exception as e:
        print(f"FC Connection Failed: {e}")
        drone = None

    cap = cv2.VideoCapture(0)
    
    # FIX: Safety check for camera opening
    if not cap.isOpened():
        print("CRITICAL ERROR: Could not open camera.")
        if drone: drone.stop()
        return
        
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    detector = cv2.aruco.ArucoDetector(aruco_dict, cv2.aruco.DetectorParameters())
    
    # FIX: Mandatory calibration data load, graceful exit if missing
    if not os.path.exists('calib_data.npz'):
        print("CRITICAL ERROR: 'calib_data.npz' not found! Please run camera_calibration.py first.")
        cap.release()
        if drone: drone.stop()
        return
        
    calib = np.load('calib_data.npz')
    cam_mtx = calib['mtx']
    cam_dist = calib['dist']

    initialize_log()

    z_filter = deque(maxlen=5) 
    last_detect_time = time.time()
    current_state = "SEARCHING"
    prev_frame_time = 0

    # FIX: Wrapped the entire main loop in a try/finally block to ensure clean resource release
    try:
        while True:
            ret, frame = cap.read()
            if not ret: 
                print("Video stream error.")
                break

            new_frame_time = time.time()
            fps = int(1 / (new_frame_time - prev_frame_time)) if prev_frame_time else 0
            prev_frame_time = new_frame_time

            corners, ids, _ = detector.detectMarkers(frame)
            
            # FIX: Telemetry validity check
            telemetry_valid = drone.is_telemetry_valid() if drone else False
            real_alt = drone.last_alt if telemetry_valid else 0.0
            
            ex, ey, est_z, confidence = 0.0, 0.0, 0.0, 0.0
            h, w = frame.shape[:2]
            frame_center = (w // 2, h // 2)

            # Process marker detection (any ID is accepted)
            if ids is not None and cam_mtx is not None:
                img_pts = corners[0][0]
                
                # Full 3D Pose Estimation
                success, rvec, tvec = cv2.solvePnP(
                    OBJ_POINTS, img_pts, cam_mtx, cam_dist, flags=cv2.SOLVEPNP_IPPE_SQUARE
                )
                
                if success:
                    # Calculate Reprojection Error for Confidence
                    proj_pts, _ = cv2.projectPoints(OBJ_POINTS, rvec, tvec, cam_mtx, cam_dist)
                    rmse = np.sqrt(np.mean(np.sum((img_pts - proj_pts[:, 0, :])**2, axis=1)))
                    confidence = max(0.0, 100.0 - (rmse * 10.0))
                    
                    # FIX: Process variables ONLY if confidence is high enough
                    if confidence >= CONFIDENCE_THRESHOLD:
                        last_detect_time = time.time()
                        
                        raw_z = tvec[2][0]
                        z_filter.append(raw_z)
                        est_z = sum(z_filter) / len(z_filter)
                        
                        tx, ty = int(np.mean(img_pts[:, 0])), int(np.mean(img_pts[:, 1]))
                        ex = (tx - frame_center[0]) / frame_center[0]
                        ey = (frame_center[1] - ty) / frame_center[1]
                        
                        is_centered = abs(ex) < CENTER_THRESHOLD and abs(ey) < CENTER_THRESHOLD
                        
                        # --- State machine logic with Telemetry validation ---
                        # Prevent LAND or PRECISION_APPROACH if telemetry is outdated/invalid
                        if is_centered and real_alt < LANDING_ALT_THRESHOLD and telemetry_valid:
                            if current_state != "LANDING":
                                current_state = "LANDING"
                                if drone: drone.set_mode('LAND') 
                        elif real_alt < PRECISION_ALT_THRESHOLD and telemetry_valid:
                            current_state = "PRECISION_APPROACH"
                        else:
                            current_state = "APPROACHING"

                        # Send est_z as the actual optical distance to the target
                        if drone: drone.send_target(ex, ey, est_z, True)
                        
                        # Draw visual tracking elements only on valid detection
                        cv2.aruco.drawDetectedMarkers(frame, corners)
                        cv2.line(frame, frame_center, (tx, ty), (0, 255, 0), 2)
                        cv2.circle(frame, (tx, ty), 5, (0, 0, 255), -1)

            # --- Fail-safe Logic ---
            if time.time() - last_detect_time > FAILSAFE_TIMEOUT:
                if current_state != "SEARCHING":
                    current_state = "SEARCHING"
                    if drone: drone.set_mode('LOITER')
                if drone: drone.send_target(0, 0, 0, False)

            log_data(current_state, real_alt, est_z, ex, ey, confidence)
            
            # --- HUD Overlay ---
            cv2.circle(frame, frame_center, 5, (255, 0, 0), -1)
            draw_text(frame, f"STATE: {current_state}", (20, 40))
            draw_text(frame, f"REAL ALT: {real_alt:.1f} cm", (20, 75))
            draw_text(frame, f"EST ALT (PnP): {est_z:.1f} cm", (20, 110))
            draw_text(frame, f"ERR X: {ex:.2f} | Y: {ey:.2f}", (20, 145))
            
            conf_color = (0, 255, 0) if confidence >= CONFIDENCE_THRESHOLD else (0, 0, 255)
            draw_text(frame, f"CONFIDENCE: {confidence:.1f}%", (20, 180), conf_color)
            
            # FIX: Telemetry validity warning
            if telemetry_valid:
                draw_text(frame, "TELEMETRY: VALID", (w - 220, 75), (0, 255, 0))
            else:
                draw_text(frame, "TELEMETRY: INVALID/LOST", (w - 250, 75), (0, 0, 255))
                
            draw_text(frame, f"FPS: {fps}", (w - 120, 40))

            if current_state == "SEARCHING" and confidence < CONFIDENCE_THRESHOLD:
                 draw_text(frame, "TARGET LOST - FAILSAFE", (20, h - 30), (0, 0, 255))

            cv2.imshow('Autonomous Landing System', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'): 
                break

    finally:
        # FIX: Ensure resources are released completely even on crash
        cap.release()
        cv2.destroyAllWindows()
        if drone: drone.stop()
        print("System shutdown cleanly.")

if __name__ == "__main__":
    main()
    