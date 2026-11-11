"""Shared camera diagnostics for the GCS. No flight commands are sent."""
import copy
import json
from pathlib import Path
import threading
import time

from config import Protocol
from ir.morse_decoder import MorseDecoder
from ir.signal_validator import SignalValidator
from ir.simulation import simulate


class VisionService:
    def __init__(self):
        self.lock = threading.RLock()
        self.stop_event = threading.Event()
        self.thread = None
        self.jpeg = None
        self.reset()

    def reset(self):
        self.data = dict(running=False, source='none', error=None, updated=None,
                         humans=None, human_detector='OpenCV HOG (experimental)',
                         ir_state=None, quality=0, centroid=None, message=None,
                         validation='waiting', confirmations=0, beacon_confirmed=False,
                         landing='BLOCKED: landing geometry unvalidated',
                         transmission_enabled=False)
        self.jpeg = None

    def snapshot(self):
        with self.lock:
            result = copy.deepcopy(self.data)
        age = None if result['updated'] is None else time.monotonic() - result['updated']
        result['age_seconds'] = age
        result['fresh'] = bool(result['running'] and age is not None and age < 2)
        if not result['fresh']:
            result.update(humans=None, ir_state=None, quality=0, centroid=None,
                          beacon_confirmed=False)
        return result

    def start(self, mode, message, unit, camera_config=None):
        protocol = Protocol(message, unit)
        if mode not in ('simulation', 'camera'):
            raise ValueError('Choose simulation or camera')
        settings = None
        if mode == 'camera':
            if not camera_config:
                raise ValueError('Provide a measured camera configuration file')
            settings = json.loads(Path(camera_config).read_text())
        with self.lock:
            if self.thread and self.thread.is_alive():
                raise ValueError('Stop the current session before starting another')
            self.reset()
            self.stop_event.clear()
            self.data.update(running=True, source=mode)
            self.thread = threading.Thread(target=self._run, args=(mode, protocol, settings), daemon=True)
            self.thread.start()

    def stop(self):
        self.stop_event.set()
        thread = self.thread
        if thread:
            thread.join(timeout=3)
        with self.lock:
            self.data['running'] = bool(thread and thread.is_alive())
            if not self.data['running']:
                self.jpeg = None

    def _run(self, mode, protocol, settings):
        source = None
        try:
            decoder, validator = MorseDecoder(protocol), SignalValidator(protocol)
            if mode == 'simulation':
                source = simulate(protocol.message, protocol.unit, protocol.confirmations)
            else:
                import cv2
                from ir.ir_camera_receiver import receive
                hog = cv2.HOGDescriptor()
                hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

                def on_frame(frame, observation):
                    # Picamera2 RGB888 arrays use OpenCV's BGR byte ordering.
                    boxes, _ = hog.detectMultiScale(frame, winStride=(8, 8), padding=(8, 8), scale=1.05)
                    display = frame.copy()
                    for x, y, w, h in boxes:
                        cv2.rectangle(display, (int(x), int(y)), (int(x+w), int(y+h)), (0, 180, 255), 2)
                    if observation['centroid']:
                        cv2.circle(display, tuple(int(v) for v in observation['centroid']), 12, (0, 255, 0), 2)
                    ok, encoded = cv2.imencode('.jpg', display)
                    if not ok:
                        raise RuntimeError('Camera preview encoding failed')
                    with self.lock:
                        self.jpeg = encoded.tobytes()
                        self.data.update(humans=len(boxes), centroid=observation['centroid'])

                source = receive(settings, on_frame=on_frame, stop_event=self.stop_event, preview=False)
            previous = None
            for timestamp, state, quality in source:
                if mode == 'simulation' and previous is not None:
                    if self.stop_event.wait(timestamp - previous):
                        break
                previous = timestamp
                if self.stop_event.is_set():
                    break
                if state is None:
                    validator.disconnect()
                decoded = decoder.feed(timestamp, state, quality)
                with self.lock:
                    self.data.update(updated=time.monotonic(), ir_state=state, quality=quality)
                    if decoded:
                        accepted, reason = validator.accept(*decoded, timestamp)
                        self.data.update(message=decoded[0], validation=reason,
                                         confirmations=validator.count,
                                         beacon_confirmed=accepted or validator.latched)
                    if state is None:
                        self.data['beacon_confirmed'] = False
                    count = self.data['humans']
                    self.data['landing'] = ('BLOCKED: person detected' if count else
                                            'BLOCKED: landing geometry unvalidated')
        except Exception as exc:
            with self.lock:
                self.data.update(error=str(exc), beacon_confirmed=False)
        finally:
            if source is not None and hasattr(source, 'close'):
                source.close()
            with self.lock:
                self.data['running'] = False
                self.jpeg = None
