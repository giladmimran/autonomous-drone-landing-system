"""All optical settings require bench calibration; there is no flight enable switch."""
from dataclasses import dataclass

@dataclass(frozen=True)
class Protocol:
    message: str
    unit: float
    tolerance: float = 0.25
    confirmations: int = 2
    confirmation_window: float = 120.0
    cooldown: float = 60.0
    minimum_quality: float = 0.8

    def __post_init__(self):
        if not self.message or any(c not in 'ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789' for c in self.message):
            raise ValueError('Use an uppercase alphanumeric single-word message')
        if self.unit <= 0 or not 0 < self.tolerance < 0.5:
            raise ValueError('Invalid timing')
        if self.confirmations < 2 or self.confirmation_window <= 0 or self.cooldown < 0:
            raise ValueError('Invalid confirmation policy')
        if not 0 <= self.minimum_quality <= 1:
            raise ValueError('Invalid quality threshold')
