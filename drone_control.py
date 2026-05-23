from pymavlink import mavutil
import threading
import time

class DroneTelemetry:
    """
    Handles MAVLink communication in a background thread to prevent lag.
    Provides methods to retrieve the latest altitude and send landing target data.
    Usage:
        drone_control = DroneTelemetry('/dev/ttyAMA0', 57600)
        drone_control.connect()
        drone_control.start()
    """
    def __init__(self, connection_string='/dev/ttyAMA0', baudrate=921600):
        self.connection_string = connection_string
        self.baudrate = baudrate
        self.master = None
        self.last_alt = 0.0
        self.last_alt_update_time = 0.0
        self.is_running = False
        self._thread = None

    def connect(self):
        """
        Establishes a MAVLink connection to the flight controller and waits for a heartbeat.
        Also requests the necessary data stream for altitude updates.
        """
        self.master = mavutil.mavlink_connection(
            self.connection_string, baud=self.baudrate, source_system=1, source_component=191)
        self.master.wait_heartbeat()
        self.request_data_stream()

    def request_data_stream(self):
        """
        Request the drone to send GLOBAL_POSITION_INT messages at 10Hz.
        This allows us to keep track of the drone's altitude in real-time.
        We only request the position stream to minimize unnecessary data and reduce latency.
        """
        self.master.mav.request_data_stream_send(self.master.target_system, self.master.target_component, mavutil.mavlink.MAV_DATA_STREAM_POSITION, 10, 1)
        
    def start(self):
        """
        Starts the background thread to listen for MAVLink messages.
        This allows us to continuously update the last known altitude without blocking the main vision processing loop.
        The thread will run until the program is terminated, ensuring we always have the latest telemetry data.
        We set the thread as a daemon so it will automatically exit when the main program ends.
        """
        self.is_running = True
        self._thread = threading.Thread(target=self._update_loop, daemon=True)
        self._thread.start()

    def stop(self):
        """
        Stops the background thread gracefully by setting the is_running flag to False.
        This allows the thread to exit its loop and terminate cleanly when the program is shutting down.
        We also join the thread to ensure it has finished before the program exits, preventing any potential issues with dangling threads.
        """      
        self.is_running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)
    
    def is_telemetry_valid(self, timeout=2.0):
        """
        Returns True if we have received an altitude update within the timeout window.
        Prevents dangerous landing decisions if the telemetry link hangs.
        """
        return (time.time() - self.last_alt_update_time) <= timeout

    def _update_loop(self):
        """
        Background thread function that continuously listens for MAVLink messages.
        When a GLOBAL_POSITION_INT message is received, it updates the last known altitude.
        This allows the main loop to access the latest altitude data without any delay, ensuring timely decision-making for landing.
        We use a non-blocking receive to ensure the thread can exit gracefully when the program ends.
        The altitude is stored in centimeters for consistency with the landing target message, which expects altitude in meters (we convert it when sending).
        """
        while self.is_running:
            msg = self.master.recv_match(blocking=False)
            if msg and msg.get_type() == 'GLOBAL_POSITION_INT':
                self.last_alt = msg.relative_alt / 10.0     # Convert mm to cm
                self.last_alt_update_time = time.time()
            time.sleep(0.01)  # Sleep briefly to prevent high CPU usage

    def set_mode(self, mode_name):
        """
        Changes the flight mode of the drone by sending a MAVLink command.
        The mode_name should be a valid mode recognized by the flight controller (e.g., 'LAND', 'LOITER').
        If the mode ID is found, we send a MAV_CMD_DO_SET_MODE command to change the drone's mode.
        This allows us to switch to LAND mode when the target is centered and the altitude is low enough, and switch to LOITER mode if the target is lost.
        """
        mode_id = self.master.mode_mapping().get(mode_name)
        if mode_id is not None:
            self.master.mav.command_long_send(
                self.master.target_system, self.master.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id,
                0, 0, 0, 0, 0
            )
            print(f"Debug: Sent command to change mode to {mode_name}.")
        else:
            print(f"Error: Mode '{mode_name}' not found in mode mapping.")

    def send_target(self, n_x, n_y, alt, is_valid=True):
        """ 
        Sends a MAVLink message with the landing target information, this function allows us to continuesly update the flight controller
            with the latest target information, enabling precise landing maneuvers when the target is detected, and providing a clear 
            indication when the target is lost.
        The n_x and n_y parameters are the normalized error values for the target's position in the camera frame, 
            which should be in the range of -1 to 1.
        The alt parameter is the estimated altitude to the target in centimeters.
        The is_valid parameter indicates whether the target detection is valid or if we are in a fail-safe state.
        The valid_flag is set to 1 if the target is valid, and 0 if we are in a fail-safe state. 
            This allows the flight controller to know whether to trust the landing target data or not.
        We send a MAVLink message of type LANDING_TARGET, which includes the normalized error values, the altitude, and the validity flag.
            The message is sent with a coordinate frame of 8 (MAV_FRAME_BODY_NED), which means the error values are relative to the
            drone's current orientation.
        The altitude is converted from centimeters to meters by dividing by 100, as the MAVLink message expects altitude in meters.
        """
        valid_flag = 1 if is_valid else 0
        # Send landing target with normalized error and altitude in meters
        self.master.mav.landing_target_send(
            0, 0, 8, n_x * 0.52, n_y * 0.52, alt/100.0, 0, 0, 0, 0, 0, [0,0,0,0], 2, valid_flag
        )
