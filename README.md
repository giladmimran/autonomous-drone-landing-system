# Vision-Based Autonomous Landing System for UAVs

A proof-of-concept autonomous landing system for UAVs using onboard computer vision, ArUco marker detection, pose estimation, and MAVLink communication with an ArduPilot-based flight controller.

The project is designed for a quadcopter platform using a Raspberry Pi companion computer, a downward-facing camera, and a CrossFlight/ArduPilot flight controller. The system detects a visual landing marker, estimates the drone’s relative position above the target, and sends landing-target data to the flight controller to support autonomous precision landing.

---

## Project Overview

Manual UAV landing can become difficult in GPS-denied environments, communication-limited areas, or situations with poor operator visibility. Existing precision landing solutions such as RTK-GPS or LiDAR may require expensive infrastructure, additional weight, or complex integration.

This project proposes a low-cost onboard solution based on:

- Raspberry Pi companion computer
- RGB camera
- OpenCV image processing
- ArUco visual markers
- Camera calibration
- `solvePnP` pose estimation
- MAVLink communication
- Telemetry validation and fail-safe logic

---

## System Architecture

The system is built around three main modules:

1. **Vision Sensing**  
   A downward-facing camera captures the landing area and detects an ArUco marker.

2. **Onboard Processing**  
   The Raspberry Pi processes the camera feed using OpenCV, estimates the marker pose using `solvePnP`, validates detection confidence, and logs flight data.

3. **Flight Control Communication**  
   The Raspberry Pi sends MAVLink `LANDING_TARGET` messages to the CrossFlight/ArduPilot flight controller and receives telemetry feedback such as altitude and system state.

Basic flow:

```text
ArUco Landing Marker
        ↓
Downward-Facing Camera
        ↓
Raspberry Pi + OpenCV
        ↓
solvePnP Pose Estimation
        ↓
MAVLink LANDING_TARGET
        ↓
CrossFlight / ArduPilot Flight Controller
        ↓
Precision Landing Response
