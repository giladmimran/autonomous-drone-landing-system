# Single-camera IR beacon landing project

The active application uses one OV5647 camera on Raspberry Pi 5 and one landing-pad
array of 20 LEDs blinking together. One capture stream feeds Morse identification
and beacon tracking. The reported roughly 50 m detection is an observation under
the user's conditions, not a guaranteed decoding or landing range.

**Development version: no flight-command path exists in the active application.**
RTL and precision landing are dry-run only. Human detection remains independent;
`~/human_detection_system/human_detection_gps.py` is not modified.

## Architecture

* `main.py`: simulation/camera entry point and structured observation logging.
* `config.py`: expected message, Morse timing and confirmation policy.
* `ir/ir_camera_receiver.py`: exactly-one-OV5647 enumeration, fixed exposure/gain,
  sensor timestamps, frame continuity checks and diagnostic preview.
* `ir/ir_detector.py`: candidate bright-region cluster, aggregate centroid,
  intensity/background contrast, ON history and OFF-period prediction.
* `ir/morse_decoder.py`: strict duration decoding; UNKNOWN invalidates a frame.
* `ir/signal_validator.py`: full-message match, repeated confirmation, cooldown
  and one accepted request per process session. Disconnection does not rearm it.
* `pixhawk/pixhawk_interface.py`: optional passive, explicitly addressed telemetry.
* `pixhawk/rtl_controller.py`: logged hypothetical decision only.
* `landing/precision_landing.py`: calibrated camera ray helper and disabled target
  preparation skeleton. No LANDING_TARGET transmissions.

ArUco detection, IDs, pose estimation, centering and automatic LAND/LOITER decisions
have been removed from the active entry point. `requirements.txt` no longer needs
`opencv-contrib-python`. General checkerboard calibration utilities are preserved.
`legacy/aruco_main.py` and `legacy/README_aruco.md` preserve the original work for
reference; **do not run the legacy flight script**. Original `drone_control.py`
still contains commands and is never imported by the new application. Marker PDF
and historical media are retained but are not required.

## Installation

Simulation and protocol tests need only Python's standard library:

```sh
python3 -m unittest discover -s tests -v
python3 main.py --simulate --message SOS --unit 0.1
```

SOS and 0.1 seconds are simulation examples, not confirmed transmitter settings.
The simulator never opens a camera or MAVLink device. It prints the decoded
message and acceptance result, followed by a blocked RTL dry-run decision when
vehicle status is absent. Events are appended to `ir_events.log`.

For camera use, install Picamera2 and OpenCV/NumPy from the package source that
supports your Pi's Debian 13 camera stack. Where those packages are available:

```sh
sudo apt install python3-picamera2 python3-opencv python3-numpy
python3 -c "from picamera2 import Picamera2; print(Picamera2.global_camera_info())"
```

Package availability on this exact Debian image has not been verified. Avoid
overlaying a working system OpenCV installation with a second pip installation.
`requirements.txt` lists Python dependencies for environments using pip; Picamera2
is supplied by the Pi camera stack. MAVLink monitoring additionally needs pymavlink
and pyserial. Verify imports in the actual Python 3.13 environment before use.

## Camera configuration and calibration

Copy `camera_config.template.json` to a local JSON file. Null entries intentionally
prevent startup until measured values are supplied. The 640x480 size is an example
and must be checked against the enumerated camera modes. Startup requires exactly
one OV5647; it does not assume a second camera.

Settings specify exposure in microseconds, analogue gain, maximum frame interval
in seconds, grayscale brightness threshold (0..255), minimum/maximum blob areas in
pixels, array search radius in pixels, phase-correlation response threshold,
maximum inter-frame motion in pixels, intensity percentile (0..100), ON/OFF contrast
against the surrounding background, maximum prediction age in seconds, ON history
length and relative exposure/gain tolerance.

Measure settings using recorded or live ground-test frames: compare beacon ON,
OFF and ambient-only images, keep exposure below the shortest pulse, avoid
saturation and measure actual frame intervals. Choose timing such that pulses
contain multiple frames. Lock exposure/gain and inspect metadata. Test sunlight,
reflections, movement and partial occlusion. No automatic threshold calibration is
claimed; diagnostic mode requires an initial measured configuration.

```sh
python3 main.py --message YOURMESSAGE --unit MEASURED_SECONDS --camera-config camera.json
```

The preview shows the feed, aggregate search circle, intensity, state, quality,
centroid and observation age information in the log. Press q to stop. Camera loss
logs an UNKNOWN observation and stops acquisition. Exposure instability or a frame
gap clears tracking and decoder continuity.

The detector clusters nearby bright components into one beacon, not separate Morse
targets. It retains bounded ON centroid history. During darkness it uses global
image phase correlation to shift the search region, only within configured motion
and age limits. This is a bench prototype: it does not yet reconstruct the full
20-LED physical geometry, authenticate IR wavelength or guarantee rejection of
reflections. Its binary quality value indicates threshold eligibility, not a
calibrated probability. OFF prediction requires textured background and must be
validated against drone motion. UNKNOWN cannot become an OFF Morse gap.

## Morse protocol and validation

`Protocol` in `config.py` exposes dot duration, timing tolerance, confirmation count,
confirmation window, cooldown and minimum quality. `--message` and `--unit` are
required: no final transmitter message or timing is invented. The initial decoder
supports uppercase alphanumeric single-word messages. Dot/dash and symbol/letter
gaps use 1/3 units. The proposed frame delimiter is at least 10 OFF units before
and after a message; `MorseDecoder(frame_units=...)` makes this adjustable.
**Confirm framing with transmitter firmware before hardware decoding.** Word gaps
and multiword messages are intentionally unsupported rather than guessed.

Starting mid-transmission, UNKNOWN observations, bad pulse/gap timings and malformed
symbols discard the frame. A missing trailing delimiter never completes a message.
Confirmation expiration restarts the count. At least two messages are required.
After acceptance, cooldown blocks immediate duplicates and a session latch blocks
later duplicates even after cooldown. Restarting the application starts a new
session; no automatic operational rearming is implemented.

## Pixhawk and RTL dry-run

Optional camera-mode arguments:

```sh
python3 main.py --message YOURMESSAGE --unit MEASURED_SECONDS --camera-config camera.json --mavlink-device /dev/serial/by-id/CONFIRMED_DEVICE --system-id CONFIRMED_ID --component-id CONFIRMED_COMPONENT
```

Monitoring does not request streams, send heartbeats, write parameters or send
commands. It accepts messages only from the configured system/component and
requires fresh telemetry. If HOME_POSITION is not broadcast, it remains missing.
GPS fix type alone is only a preliminary check, not a complete navigation-health
assessment. RTL capability and appropriate vehicle state remain unapproved, so
real monitoring deliberately blocks RTL eligibility until firmware-specific safety
logic is separately developed and tested. Disconnected/stale telemetry fails closed.
RTL returns to configured home, not automatically to the IR pad.

## Precision landing and shared blinking geometry

ON-frame centroid history is diagnostic. OFF estimates are predictions, not new
measurements. Calibration must match resolution/crop; lens distortion, camera to
body rotation/translation, attitude, timestamp alignment, valid range and ground
geometry are required before metric position can be prepared. Pixhawk relative
altitude alone is not necessarily distance to the pad.

Evaluate dedicated sustained-ON landing windows or a separate landing transmission
phase. Long OFF Morse gaps can exceed usable target-estimate freshness. No landing
window schedule is assumed. Landing validity remains false throughout this version.

Future integration must follow the supported LANDING_TARGET coordinate frame,
distance and position-validity requirements:
https://mavlink.io/en/services/landing_target.html
https://ardupilot.org/dev/docs/mavlink-precision-landing.html

## Verification and remaining hardware work

Protocol tests cover valid/repeated messages, incomplete frames, noise, wrong pulse
and character timing, low quality, receiver loss, confirmation timeout, cooldown,
duplicates, timestamp ordering and stale/invalid Pixhawk status. Camera tests use
fakes where hardware is unavailable; real image-processing validation remains open.
Development tests run on Windows Python 3.10, not the target Pi/Python 3.13 stack.

Still required: camera filter/lens and mounting details; LED wavelength, physical
coordinates and driver/controller circuit; confirmed Morse framing/timing/message;
actual camera enumeration and recordings; firmware/vehicle identity; telemetry
availability; range/ground geometry; measured thresholds and controlled ground tests.
Wiring is intentionally unspecified until LED power/driver hardware is confirmed.
