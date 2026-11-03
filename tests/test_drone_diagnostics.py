import unittest
from types import SimpleNamespace
from drone_control import DroneTelemetry

def message(kind, **fields):
    return SimpleNamespace(get_type=lambda: kind, get_srcSystem=lambda: 1,
                           get_srcComponent=lambda: 1, **fields)

class DiagnosticTests(unittest.TestCase):
    def monitor(self):
        monitor = DroneTelemetry('unused')
        monitor.ingest(message('HEARTBEAT', autopilot=3, type=2, base_mode=0), 10)
        return monitor

    def test_sensor_masks_and_staleness(self):
        monitor = self.monitor()
        mask = 1 | 2 | 8 | (1 << 28)
        monitor.ingest(message('SYS_STATUS', onboard_control_sensors_present=mask,
                               onboard_control_sensors_enabled=mask,
                               onboard_control_sensors_health=mask, voltage_battery=12000), 10)
        status = monitor.snapshot(11)
        self.assertEqual(status['barometer'], 'HEALTHY')
        self.assertEqual(status['imu']['gyro'], 'HEALTHY')
        self.assertEqual(status['battery_voltage_v'], 12)
        self.assertIn('PASS', status['prearm'])
        self.assertEqual(monitor.snapshot(16)['heartbeat'], 'MISSING/STALE')
        self.assertIn('UNKNOWN', monitor.snapshot(16)['prearm'])

    def test_missing_disabled_unhealthy(self):
        monitor = self.monitor()
        self.assertIn('UNKNOWN', monitor.snapshot(11)['prearm'])
        monitor.ingest(message('SYS_STATUS', onboard_control_sensors_present=11,
                               onboard_control_sensors_enabled=3,
                               onboard_control_sensors_health=1, voltage_battery=65535), 10)
        status = monitor.snapshot(11)
        self.assertIn('DISABLED', status['barometer'])
        self.assertEqual(status['imu']['accelerometer'], 'UNHEALTHY')
        self.assertIsNone(status['battery_voltage_v'])

    def test_battery_fallback(self):
        monitor = self.monitor()
        monitor.ingest(message('BATTERY_STATUS', voltages=[4000, 4000, 4000, 65535]), 10)
        self.assertEqual(monitor.snapshot(11)['battery_voltage_v'], 12)

    def test_gcs_not_selected(self):
        monitor = DroneTelemetry('unused')
        monitor.ingest(message('HEARTBEAT', autopilot=8, type=6, base_mode=0), 10)
        self.assertIsNone(monitor.system_id)

    def test_commands_fail_closed(self):
        monitor = self.monitor()
        for method in (monitor.set_mode, monitor.send_target, monitor.request_data_stream):
            with self.assertRaises(RuntimeError):
                method()

    def test_other_vehicle_filtered(self):
        monitor = self.monitor()
        other = message('GPS_RAW_INT', fix_type=3, satellites_visible=10)
        other.get_srcSystem = lambda: 2
        monitor.ingest(other, 10)
        self.assertIn('UNKNOWN', monitor.snapshot(11)['gps'])
