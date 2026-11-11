"""Passive Pixhawk diagnostics for indoor hover preparation.

No MAVLink transmissions, parameter writes, arming or mode changes. Run:
  python3 drone_control.py --device /dev/serial/by-id/CONFIRMED_DEVICE
AltHold provides altitude hold, not horizontal position hold or hover approval.
"""
import argparse
from collections import deque
import json
import threading
import time


class DroneTelemetry:
    def __init__(self, connection_string, baudrate=115200, *, timeout=10.0,
                 stale_after=5.0, system_id=None, component_id=1):
        if timeout <= 0 or stale_after <= 0:
            raise ValueError('Timeouts must be positive')
        self.connection_string, self.baudrate = connection_string, baudrate
        self.timeout, self.stale_after = timeout, stale_after
        self.system_id, self.component_id = system_id, component_id
        self.master = None
        self.messages = {}
        self.notices = deque(maxlen=10)
        self.is_running = False
        self.error = None
        self._thread = None
        self._lock = threading.RLock()
        self._stop = threading.Event()
        self._mavutil = None
        self.last_alt = 0.0
        self.last_alt_update_time = None

    def connect(self):
        from pymavlink import mavutil
        self._mavutil = mavutil
        try:
            self.master = mavutil.mavlink_connection(
                self.connection_string, baud=self.baudrate, autoreconnect=False,
                source_system=255, source_component=190)
            deadline = time.monotonic() + self.timeout
            while time.monotonic() < deadline:
                self.poll()
                if self._fresh('HEARTBEAT', time.monotonic()) is not None:
                    return
                time.sleep(0.02)
            raise TimeoutError('No matching ArduPilot autopilot heartbeat before timeout')
        except Exception as exc:
            self.error = str(exc)
            self.stop()
            raise

    def ingest(self, message, now=None):
        """Receive-only message handling, also usable with recorded test messages."""
        now = time.monotonic() if now is None else now
        with self._lock:
            kind = message.get_type()
            if message.get_srcComponent() != self.component_id:
                return
            if self.system_id is None:
                if (kind != 'HEARTBEAT' or message.autopilot != 3 or message.type in (5, 6, 18, 26, 27)):
                    return
                self.system_id = message.get_srcSystem()
            if message.get_srcSystem() != self.system_id:
                return
            if kind == 'HEARTBEAT' and message.autopilot != 3:
                return
            self.messages[kind] = (now, message)
            if kind == 'STATUSTEXT':
                text = message.text
                if isinstance(text, bytes):
                    text = text.decode('utf-8', errors='replace')
                self.notices.append((now, str(text).rstrip('\x00')))
            elif kind == 'GLOBAL_POSITION_INT':
                self.last_alt = message.relative_alt / 10.0
                self.last_alt_update_time = now

    def poll(self):
        if self.master is None:
            return
        for _ in range(200):
            message = self.master.recv_match(blocking=False)
            if message is None:
                break
            self.ingest(message)

    def _fresh(self, kind, now):
        entry = self.messages.get(kind)
        return entry[1] if entry and 0 <= now-entry[0] <= self.stale_after else None

    @staticmethod
    def _sensor(status, mask):
        if status is None:
            return 'UNKNOWN (missing/stale SYS_STATUS)'
        if status.onboard_control_sensors_present & mask != mask:
            return 'NOT PRESENT / NOT REPORTED'
        if status.onboard_control_sensors_enabled & mask != mask:
            return 'DISABLED / NOT ENABLED'
        return 'HEALTHY' if status.onboard_control_sensors_health & mask == mask else 'UNHEALTHY'

    def snapshot(self, now=None):
        now = time.monotonic() if now is None else now
        with self._lock:
            heartbeat = self._fresh('HEARTBEAT', now)
            status = self._fresh('SYS_STATUS', now)
            gps = self._fresh('GPS_RAW_INT', now)
            voltage = None
            if status is not None and status.voltage_battery not in (0, 65535):
                voltage = status.voltage_battery / 1000.0
            battery = self._fresh('BATTERY_STATUS', now)
            if voltage is None and battery is not None:
                cells = [v for v in battery.voltages if v != 65535]
                if cells and all(v > 0 for v in cells):
                    voltage = sum(cells) / 1000.0
            mode = 'UNKNOWN (missing/stale heartbeat)'
            modes = []
            if heartbeat is not None and self._mavutil is not None:
                mode = self._mavutil.mode_string_v10(heartbeat)
                if self.master is not None:
                    modes = sorted((self.master.mode_mapping() or {}).keys())
            prearm = self._sensor(status, 1 << 28)
            prearm = {'HEALTHY': 'REPORTED PASS (not authorization to arm)',
                      'UNHEALTHY': 'REPORTED FAIL'}.get(prearm, 'UNKNOWN: ' + prearm)
            notices = [text for stamp, text in self.notices if now-stamp <= 30]
            return {
                'mavlink_connection': 'OPEN' if self.master is not None and not self.error else 'CLOSED/ERROR',
                'connection_error': self.error,
                'heartbeat': 'RECEIVED/FRESH' if heartbeat else 'MISSING/STALE',
                'vehicle_ids': [self.system_id, self.component_id],
                'flight_mode': mode,
                'armed': bool(heartbeat.base_mode & 128) if heartbeat else None,
                'gps': {'fix_type': gps.fix_type, 'satellites': gps.satellites_visible} if gps else 'UNKNOWN (missing/stale)',
                'imu': {'gyro': self._sensor(status, 1), 'accelerometer': self._sensor(status, 2)},
                'barometer': self._sensor(status, 8),
                'battery_voltage_v': voltage,
                'battery_note': 'SYS_STATUS or BATTERY_STATUS voltage; null means unavailable, not zero volts',
                'prearm': prearm,
                'recent_autopilot_text': notices,
                'gps_required': 'AltHold itself does not require GPS position; configured arming/EKF/fence requirements UNKNOWN (no parameters requested)',
                'available_modes_library_mapping': modes,
                'mode_note': 'Library mapping is not firmware-advertised availability or permission to enter a mode',
                'althold_assessment': 'NOT FLIGHT APPROVAL: inspect sensor health, estimator, RC, pre-arm messages and setup; no horizontal hold',
                'control': 'MONITOR ONLY; flight commands disabled',
            }

    def start(self):
        if self.master is None:
            raise RuntimeError('Connect before starting')
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self.is_running = True
        self._thread = threading.Thread(target=self._update_loop, daemon=True)
        self._thread.start()

    def _update_loop(self):
        try:
            while not self._stop.is_set():
                self.poll()
                self._stop.wait(0.02)
        except Exception as exc:
            self.error = str(exc)
        finally:
            self.is_running = False

    def stop(self):
        self._stop.set()
        if self._thread and self._thread is not threading.current_thread():
            self._thread.join(timeout=2)
        if self.master is not None:
            self.master.close()
            self.master = None
        self.is_running = False

    def is_telemetry_valid(self, timeout=2.0):
        return self.last_alt_update_time is not None and time.monotonic()-self.last_alt_update_time <= timeout

    def set_mode(self, *args, **kwargs):
        raise RuntimeError('Flight-mode commands disabled: monitor-only interface')

    def send_target(self, *args, **kwargs):
        raise RuntimeError('Landing target transmission disabled: monitor-only interface')

    def request_data_stream(self, *args, **kwargs):
        raise RuntimeError('Stream requests disabled: passive monitoring only')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--device', required=True, help='Verified USB serial path; no default UART assumption')
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument('--timeout', type=float, default=10)
    parser.add_argument('--stale-after', type=float, default=5)
    parser.add_argument('--interval', type=float, default=1)
    parser.add_argument('--system-id', type=int)
    parser.add_argument('--component-id', type=int, default=1)
    args = parser.parse_args()
    if args.interval <= 0:
        parser.error('--interval must be positive')
    drone = DroneTelemetry(args.device, args.baud, timeout=args.timeout,
                           stale_after=args.stale_after, system_id=args.system_id,
                           component_id=args.component_id)
    try:
        drone.connect()
        while True:
            drone.poll()
            print(json.dumps(drone.snapshot(), indent=2), flush=True)
            time.sleep(args.interval)
    except KeyboardInterrupt:
        return 0
    except Exception as exc:
        drone.error = str(exc)
        print(json.dumps(drone.snapshot(), indent=2), flush=True)
        return 1
    finally:
        drone.stop()


if __name__ == '__main__':
    raise SystemExit(main())
