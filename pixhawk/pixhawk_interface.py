import time

class PixhawkMonitor:
    """Never requests streams or sends commands. Missing telemetry fails closed."""
    def __init__(self, device, system_id, component_id):
        from pymavlink import mavutil
        self.link = mavutil.mavlink_connection(device)
        self.ids = system_id, component_id
        self.messages = {}

    def poll(self):
        for _ in range(100):
            message = self.link.recv_match(blocking=False)
            if message is None:
                break
            if (message.get_srcSystem(), message.get_srcComponent()) == self.ids:
                self.messages[message.get_type()] = (time.monotonic(), message)

    def status(self, now=None):
        now = time.monotonic() if now is None else now
        def fresh(name):
            value = self.messages.get(name)
            return value[1] if value and now - value[0] <= 3 else None
        heartbeat, gps, home = fresh('HEARTBEAT'), fresh('GPS_RAW_INT'), fresh('HOME_POSITION')
        return {'connected': heartbeat is not None, 'gps': bool(gps and gps.fix_type >= 3),
                'home': home is not None, 'mode': heartbeat.custom_mode if heartbeat else None,
                'rtl_supported': False, 'state_approved': False}

    def close(self):
        self.link.close()
