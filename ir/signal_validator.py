class SignalValidator:
    """One request per session; connection loss cannot rearm acceptance."""
    def __init__(self, config):
        self.config = config
        self.count, self.first, self.latched = 0, None, False
        self.last_request = float('-inf')

    def disconnect(self):
        self.count, self.first = 0, None

    def accept(self, message, quality, now):
        if message != self.config.message or quality < self.config.minimum_quality:
            self.disconnect()
            return False, 'message_or_quality_rejected'
        if now - self.last_request < self.config.cooldown:
            return False, 'cooldown'
        if self.latched:
            return False, 'duplicate_session'
        if self.first is None or now - self.first > self.config.confirmation_window:
            self.first, self.count = now, 0
        self.count += 1
        if self.count < self.config.confirmations:
            return False, 'confirmation_pending'
        self.last_request, self.latched = now, True
        return True, 'validated'
