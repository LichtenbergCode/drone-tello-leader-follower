import socket
import time
import logging
from colorlog import ColoredFormatter
import threading

class Tello:
    def __init__(self, 
                 tello_ip="192.168.10.1", 
                 command_port=8889,
                 state_port=8890,
                 local_port=9000):
        self.tello_ip = tello_ip
        self.command_port = command_port
        self.local_port = local_port
        self.state_port = state_port
        self.command_address = (self.tello_ip, self.command_port)

        # Socket comandos
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("", self.local_port))

        # Socket estado
        self.state_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.state_sock.bind(("", self.state_port))
        
        # logger
        formatter = ColoredFormatter(
            "%(log_color)s%(levelname)s:%(name)s:%(message)s",
            log_colors={
                "DEBUG": "cyan",
                "INFO": "green",
                "WARNING": "yellow",
                "ERROR": "red",
                "CRITICAL": "bold_red",
            }
        )
        handler = logging.StreamHandler()
        handler.setFormatter(formatter)

        self.logger = logging.getLogger(__name__)
        self.logger.setLevel(logging.INFO)
        self.logger.addHandler(handler)

        # Flags
        self.is_flying = False
        self.is_connected = False
        self.streamon_activation = False
        self.get_state = False

        # Estado actual
        self.current_state = {}
        self.pitch = 0
        self.roll = 0
        self.yaw = 0
        self.vgx = 0
        self.vgy = 0
        self.vgz = 0
        self.templ = 0
        self.temph = 0
        self.tof = 10
        self.h = 0 
        self.bat  = 0
        self.baro = 0.0
        self.time = 0
        self.agx = 0.0
        self.agy = 0.0
        self.agz = 0.0

    def send_command(self, cmd, retries=3, timeout=2.0):
        """
        Sends a command to the Tello and waits for a response.
        Retries up to `retries` times if no valid response is received.
        """
        for attempt in range(1, retries + 1):
            self.logger.info(f"[{attempt}/{retries}] Sending: {cmd}")
            try:
                self.sock.sendto(cmd.encode("utf-8"), self.command_address)
                self.sock.settimeout(timeout)

                response, _ = self.sock.recvfrom(1024)
                decoded = response.decode("utf-8", errors="ignore").strip().lower()

                if decoded == "ok":
                    self.logger.info(f"{cmd} : ✅ Success")
                    return True, decoded
                elif decoded == "error":
                    self.logger.warning(f"{cmd} : ⚠️ Error response received")
                    return False, decoded
                else:
                    self.logger.warning(f"{cmd} : ⚠️ Unexpected response '{decoded}'")
                    # try again

            except socket.timeout:
                self.logger.warning(f"{cmd} : ⏱️ Timeout waiting for response (attempt {attempt})")
            except UnicodeDecodeError as e:
                self.logger.error(f"{cmd} : ❌ Failed to decode response ({e})")

            # Short delay before retrying
            time.sleep(0.5)

        self.logger.error(f"{cmd} : ❌ Failed after {retries} attempts")
        return False, None

    def recv_state(self):
        def try_convert(v):
            try:
                if "." in v:
                    return float(v)
                return int(v)
            except ValueError:
                return v

        while self.is_connected:
            data, _ = self.state_sock.recvfrom(1024)
            state = data.decode("utf-8")
            values = dict(item.split(":") for item in state.strip().split(";") if item)
            values = {k: try_convert(v) for k, v in values.items()}

            self.current_state = values
            for k in values:
                if hasattr(self, k):
                    setattr(self, k, values[k])

            self.get_state = True
            self.logger.debug("Telemetry updated")

    def connect(self):
        if not self.is_connected:
            success, _ = self.send_command("command") # SDK Activation
            time.sleep(0.3)
            if success:
                self.is_connected = True
                threading.Thread(target=self.recv_state, daemon=True).start()

    def start_video(self):
        self.connect()
        time.sleep(2)
        self.send_command("streamon") 
        time.sleep(0.5)
    
    def tk_off(self):
    # stop any rc stream before takeoff
        self.send_command("rc 0 0 0 0")
        ok, _ = self.send_command("takeoff")
        if ok:
            self.is_flying = True
        return ok

    def land(self):
        """
        Robust landing:
        - Stop RC streaming.
        - Ensure SDK mode is active.
        - Try land with retries (handled inside send_command).
        - Update is_flying flag.
        """
        # If we have telemetry, skip if already on ground
        if self.get_state and isinstance(self.h, int) and self.h <= 0 and not self.is_flying:
            self.logger.info("Already on ground; skipping land")
            return True

        # 1) Ensure RC is neutral
        self.send_command("rc 0 0 0 0")
        time.sleep(0.2)

        # 2) Ensure connected / SDK mode alive
        if not self.is_connected:
            self.logger.warning("Not connected; re-entering SDK mode")
            self.connect()
            time.sleep(0.2)
        else:
            # Re-assert command mode in case it dropped silently
            self.send_command("command")
            time.sleep(0.1)

        # 3) Try to land
        ok, _ = self.send_command("land")
        if ok:
            self.is_flying = False
            return True

        # 4) Second try after ensuring hover
        self.logger.warning("Land failed once; ensuring hover and retrying")
        self.send_command("rc 0 0 0 0")
        time.sleep(0.3)
        ok, _ = self.send_command("land")
        if ok:
            self.is_flying = False
            return True

        self.logger.error("Land failed after retries")
        return False
    
    def send_rc_command(self, roll=0, pitch=0, throttle=0, yaw=0):
        self.send_command(f"rc {roll} {pitch} {throttle} {yaw}")

    def get_current_state(self):
        if self.get_state: 
            return True, self.current_state
        else: 
            return False, None
