"""Single-camera acquisition, metadata checks and bench diagnostics."""
import logging
import time
from .ir_detector import IRDetector

REQUIRED = ('size', 'exposure_us', 'gain', 'max_frame_gap', 'brightness_threshold',
            'min_blob_area', 'max_blob_area', 'array_radius_pixels', 'motion_response',
            'max_motion_pixels', 'intensity_percentile', 'on_contrast', 'off_contrast',
            'max_off_seconds', 'history_size', 'exposure_tolerance')

def receive(settings, *, on_frame=None, stop_event=None, preview=True):
    import cv2
    from picamera2 import Picamera2
    missing = set(REQUIRED) - settings.keys()
    if missing:
        raise ValueError('Missing measured camera settings: ' + ', '.join(sorted(missing)))
    for key in REQUIRED:
        if key != 'size' and (not isinstance(settings[key], (int, float)) or settings[key] <= 0):
            raise ValueError('Provide a positive measured value for ' + key)
    if len(settings['size']) != 2 or any(not isinstance(v, int) or v <= 0 for v in settings['size']):
        raise ValueError('size must contain two positive integer dimensions')
    if not isinstance(settings['history_size'], int) or not 0 < settings['intensity_percentile'] <= 100:
        raise ValueError('Invalid history size or percentile')
    if settings['off_contrast'] >= settings['on_contrast']:
        raise ValueError('OFF contrast must be below ON contrast')
    cameras = Picamera2.global_camera_info()
    logging.info('Camera enumeration: %r', cameras)
    if len(cameras) != 1 or 'ov5647' not in str(cameras[0]).lower():
        raise RuntimeError('Require exactly one enumerated OV5647; inspect camera configuration')
    camera = Picamera2(0)
    started = False
    previous = None
    try:
        camera.configure(camera.create_video_configuration(main={'size': tuple(settings['size']), 'format': 'RGB888'}))
        for control in ('AeEnable', 'ExposureTime', 'AnalogueGain'):
            if control not in camera.camera_controls:
                raise RuntimeError('Unsupported camera control: ' + control)
        camera.set_controls({'AeEnable': False, 'ExposureTime': settings['exposure_us'], 'AnalogueGain': settings['gain']})
        camera.start()
        started = True
        detector, previous = IRDetector(settings), None
        while stop_event is None or not stop_event.is_set():
            request = camera.capture_request()
            try:
                frame, metadata = request.make_array('main'), request.get_metadata()
            finally:
                request.release()
            # Sensor timestamp is tied to capture rather than processing speed.
            now = metadata['SensorTimestamp'] / 1e9
            exposure = metadata.get('ExposureTime')
            gain = metadata.get('AnalogueGain')
            stable = (exposure is not None and gain is not None and
                      abs(exposure/settings['exposure_us']-1) <= settings['exposure_tolerance'] and
                      abs(gain/settings['gain']-1) <= settings['exposure_tolerance'])
            if not stable or (previous is not None and now-previous > settings['max_frame_gap']):
                detector = IRDetector(settings)
                observation = detector.result(None, 0, 0, False)
            else:
                observation = detector.process(frame, now)
            previous = now
            logging.info('Camera observation: %r metadata: %r', observation, metadata)
            if on_frame is not None:
                on_frame(frame, observation)
            if not preview:
                yield now, observation['state'], observation['quality']
                continue
            display = frame.copy()
            if observation['centroid']:
                center = tuple(int(v) for v in observation['centroid'])
                cv2.circle(display, center, int(settings['array_radius_pixels']), (0, 255, 0), 1)
            cv2.putText(display, str(observation), (5, 25), cv2.FONT_HERSHEY_SIMPLEX, .4, (0, 255, 0), 1)
            cv2.imshow('IR diagnostics - q to quit', display)
            yield now, observation['state'], observation['quality']
            if cv2.waitKey(1) & 0xff == ord('q'):
                return
    except Exception:
        logging.exception('Camera disconnected or acquisition failed')
        yield (previous + 1e-6 if previous is not None else time.monotonic()), None, 0
        raise
    finally:
        if started:
            camera.stop()
        camera.close()
        if preview:
            cv2.destroyAllWindows()
