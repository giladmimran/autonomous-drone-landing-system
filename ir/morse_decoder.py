"""Strict streaming Morse: require ten-unit OFF framing, discard UNKNOWN.

Single-word alphanumeric protocol only. Ten units is a proposed configurable
protocol convention, not a claim about the transmitter firmware.
"""
TABLE = dict(zip('ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789',
    '.- -... -.-. -.. . ..-. --. .... .. .--- -.- .-.. -- -. --- .--. --.- .-. ... - ..- ...- .-- -..- -.-- --.. ----- .---- ..--- ...-- ....- ..... -.... --... ---.. ----.'.split()))
REVERSE = {v: k for k, v in TABLE.items()}

class MorseDecoder:
    def __init__(self, config, frame_units=10):
        self.config, self.frame_units = config, frame_units
        self.state = self.since = self.last_time = None
        self.reset()

    def reset(self):
        self.synced, self.symbol, self.text, self.quality = False, '', '', 1.0

    def near(self, duration, units):
        return abs(duration / self.config.unit - units) <= units * self.config.tolerance

    def letter(self):
        if self.symbol not in REVERSE:
            self.reset()
            return False
        self.text += REVERSE[self.symbol]
        self.symbol = ''
        if len(self.text) > len(self.config.message):
            self.reset()
            return False
        return True

    def feed(self, timestamp, state, quality=1.0):
        import math
        if not math.isfinite(timestamp) or (self.last_time is not None and timestamp <= self.last_time):
            raise ValueError('Timestamps must be finite and strictly increasing')
        self.last_time = timestamp
        if state is None or quality < self.config.minimum_quality:
            self.reset()
            self.state, self.since = None, timestamp
            return None
        if self.state is None:
            self.state, self.since = state, timestamp
            return None
        self.quality = min(self.quality, quality)
        elapsed = timestamp - self.since
        if state == self.state:
            if not state and elapsed >= self.frame_units * self.config.unit:
                result = None
                if self.synced and self.symbol and self.letter():
                    result = self.text, self.quality
                self.reset()
                self.synced = True
                return result
            if state and elapsed > 3 * self.config.unit * (1 + self.config.tolerance):
                self.reset()
            return None
        previous = self.state
        self.state, self.since = state, timestamp
        if previous:
            if not self.synced:
                return None
            if self.near(elapsed, 1):
                self.symbol += '.'
            elif self.near(elapsed, 3):
                self.symbol += '-'
            else:
                self.reset()
            if len(self.symbol) > 5:
                self.reset()
        elif elapsed >= self.frame_units * self.config.unit:
            result = None
            if self.synced and self.symbol and self.letter():
                result = self.text, self.quality
            self.reset()
            self.synced = True
            return result
        elif self.synced:
            if self.near(elapsed, 3):
                self.letter()
            elif not self.near(elapsed, 1):
                self.reset()
        return None
