import unittest
from config import Protocol
from ir.morse_decoder import MorseDecoder
from ir.signal_validator import SignalValidator
from ir.simulation import simulate
from pixhawk.rtl_controller import dry_run
from pixhawk.pixhawk_interface import PixhawkMonitor
from types import SimpleNamespace

class PipelineTests(unittest.TestCase):
    def decode(self, observations):
        decoder = MorseDecoder(Protocol('SOS', .1))
        return [result[0] for row in observations if (result := decoder.feed(*row))]

    def test_valid_repeated_message(self):
        self.assertEqual(self.decode(simulate('SOS', .1)), ['SOS', 'SOS'])

    def test_incomplete(self):
        self.assertEqual(self.decode(list(simulate('SOS', .1, 1))[:-1]), [])

    def test_wrong_timing(self):
        self.assertEqual(self.decode(simulate('SOS', .17, 1)), [])

    def test_receiver_loss(self):
        rows = list(simulate('SOS', .1, 1))
        rows[5] = (rows[5][0], None, 0)
        self.assertEqual(self.decode(rows), [])

    def test_noise(self):
        self.assertEqual(self.decode([(0, False, 1), (2, True, 1), (2.02, False, 1), (4, False, 1)]), [])

    def test_background_interference(self):
        self.assertEqual(self.decode((t, s, .1) for t, s, q in simulate('SOS', .1)), [])

    def test_invalid_message(self):
        validator = SignalValidator(Protocol('SOS', .1))
        self.assertFalse(validator.accept('SO', 1, 1)[0])

    def test_confirmation_duplicate_cooldown(self):
        validator = SignalValidator(Protocol('SOS', .1))
        self.assertFalse(validator.accept('SOS', 1, 1)[0])
        self.assertTrue(validator.accept('SOS', 1, 2)[0])
        self.assertEqual(validator.accept('SOS', 1, 3)[1], 'cooldown')
        validator.disconnect()
        self.assertEqual(validator.accept('SOS', 1, 100)[1], 'duplicate_session')

    def test_confirmation_timeout(self):
        validator = SignalValidator(Protocol('SOS', .1, confirmation_window=2))
        validator.accept('SOS', 1, 1)
        self.assertFalse(validator.accept('SOS', 1, 5)[0])

    def test_character_gap(self):
        self.assertEqual(self.decode([(0, False, 1), (2, True, 1), (2.1, False, 1), (2.3, True, 1), (2.4, False, 1), (4, False, 1)]), [])

    def test_timestamp_order(self):
        decoder = MorseDecoder(Protocol('SOS', .1))
        decoder.feed(1, False)
        with self.assertRaises(ValueError):
            decoder.feed(1, True)

    def test_pixhawk_health(self):
        monitor = PixhawkMonitor.__new__(PixhawkMonitor)
        monitor.messages = {}
        self.assertFalse(monitor.status(10)['connected'])
        monitor.messages = {'HEARTBEAT': (10, SimpleNamespace(custom_mode=1)), 'GPS_RAW_INT': (10, SimpleNamespace(fix_type=1))}
        status = monitor.status(10)
        self.assertFalse(status['gps'])
        self.assertFalse(status['home'])
        self.assertIn('blocked', dry_run(status))
        self.assertFalse(monitor.status(14)['connected'])

if __name__ == '__main__':
    unittest.main()
