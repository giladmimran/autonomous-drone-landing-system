import time
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from config import Protocol
from vision_service import VisionService


class VisionTests(unittest.TestCase):
    def test_simulation_decodes_without_camera_or_people(self):
        service = VisionService()
        with patch.object(service.stop_event, 'wait', return_value=False):
            service._run('simulation', Protocol('SOS', .1), None)
        self.assertEqual(service.data['message'], 'SOS')
        self.assertEqual(service.data['validation'], 'validated')
        self.assertIsNone(service.data['humans'])
        self.assertFalse(service.snapshot()['beacon_confirmed'])
        self.assertFalse(service.snapshot()['transmission_enabled'])

    def test_stale_frames_cannot_report_clear_area_or_valid_beacon(self):
        service = VisionService()
        service.data.update(running=True, updated=time.monotonic()-3,
                            humans=0, ir_state=True, beacon_confirmed=True)
        status = service.snapshot()
        self.assertFalse(status['fresh'])
        self.assertIsNone(status['humans'])
        self.assertIsNone(status['ir_state'])
        self.assertFalse(status['beacon_confirmed'])

    def test_invalid_source_and_protocol_rejected_before_thread(self):
        service = VisionService()
        with self.assertRaises(ValueError):
            service.start('unknown', 'SOS', .1)
        with self.assertRaises(ValueError):
            service.start('simulation', 'sos', .1)
        self.assertIsNone(service.thread)

    def test_camera_failure_is_visible(self):
        service = VisionService()
        fake = SimpleNamespace(HOGDescriptor=lambda: SimpleNamespace(setSVMDetector=lambda _: None),
                               HOGDescriptor_getDefaultPeopleDetector=lambda: None)
        with patch.dict(sys.modules, {'cv2': fake}), patch('ir.ir_camera_receiver.receive', side_effect=RuntimeError('camera unavailable')):
            service._run('camera', Protocol('SOS', .1), {})
        self.assertIn('camera unavailable', service.snapshot()['error'])
        self.assertFalse(service.snapshot()['running'])
