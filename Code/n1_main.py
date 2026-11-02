"""Camera reception, detection and control connection operate in separate threads."""
import socket
import struct
import threading
import time
import cv2
import numpy as np
from PyQt6.QtCore import QThread
from n2_vision import VisionNode
from n3_PID import PIDNode

MAX_JPEG = 8 * 1024 * 1024
VALID = set('WASDX')

class RobotState:
    # The cloud reply is only actionable for at most this many seconds from
    # the source image, including transport and inference time.
    MAX_CAPTURE_AGE = 1.30
    CAMERA_TIMEOUT = 0.45

    def __init__(self):
        self.lock = threading.Lock()
        self.armed = False
        self.auto = False
        self.forward_allowed = False
        self.manual = 'X'
        self.last_camera = 0.0
        self.last_capture = 0.0
        self.api_age = float('inf')
        self.confirmed = False
        self.verified = 0
        self.decision = 'X'
        self.label = 'WAITING FOR CAMERA'
        self.connected = False
        self.api_key = ''
        self.last_seq = 0
        self.block_reason = 'DISARMED (M)'

    def set_api_key(self, key):
        with self.lock:
            if self.api_key != key:
                self.auto = False
            self.api_key = key
            self.confirmed = False
            self.last_capture = 0.0
            self.last_seq = 0
            self.decision = 'X'

    def set_manual(self, cmd):
        with self.lock:
            self.manual = cmd if cmd in VALID else 'X'
            self.auto = False

    def toggle_arm(self):
        with self.lock:
            self.armed = not self.armed
            if not self.armed:
                self.auto = False
                self.forward_allowed = False
                self.manual = 'X'
            return self.armed

    def toggle_auto(self):
        with self.lock:
            self.auto = not self.auto if self.armed else False
            self.manual = 'X'
            return self.auto

    def toggle_forward(self):
        with self.lock:
            self.forward_allowed = not self.forward_allowed if self.armed else False
            return self.forward_allowed

    def estop(self):
        with self.lock:
            self.armed = False
            self.auto = False
            self.forward_allowed = False
            self.manual = 'X'
            self.decision = 'X'
            self.confirmed = False

    def update_vision(self, decision, data):
        with self.lock:
            now = time.monotonic()
            self.last_camera = now
            self.last_seq = data.get('seq', 0)
            self.api_age = float(data.get('capture_age', float('inf')))
            self.verified = int(data.get('verified', 0))
            self.confirmed = bool(data.get('confirmed', False))
            # This value does NOT advance between cloud replies: recomputing
            # now - increasing age recovers the same capture timestamp.
            self.last_capture = now-self.api_age if self.confirmed else 0.0
            self.decision = decision.cmd
            self.label = decision.label

    def command(self):
        with self.lock:
            now = time.monotonic()
            if not self.armed:
                self.block_reason = 'DISARMED (M)'
                return 'X'
            if not self.connected:
                self.block_reason = 'CONTROL LINK DOWN'
                return 'X'
            if self.auto:
                if now-self.last_camera > self.CAMERA_TIMEOUT:
                    self.block_reason = 'CAMERA FRAMES LOST'
                    return 'X'
                if not self.confirmed:
                    if self.api_age >= self.MAX_CAPTURE_AGE:
                        self.block_reason = 'STALE / NO API RESULT'
                    else:
                        self.block_reason = f'WAITING CLOUD CONFIRMATION {self.verified}/2'
                    return 'X'
                if now-self.last_capture > self.MAX_CAPTURE_AGE:
                    self.block_reason = 'STALE SOURCE FRAME'
                    return 'X'
                if self.decision == 'X':
                    self.block_reason = self.label
                    return 'X'
                if self.decision == 'W' and not self.forward_allowed:
                    self.block_reason = 'FORWARD LOCKED (F)'
                    return 'X'
                self.block_reason = 'AUTO ACTIVE | FRESH TARGET'
                return self.decision
            self.block_reason = 'AUTO OFF (T)'
            return self.manual

class CameraWorker(QThread):
    def __init__(self, bus, state, host, port=5001):
        super().__init__()
        self.bus, self.state, self.host, self.port = bus, state, host, port
        self.running = True
        self.sock = None
        self.vision = VisionNode()
        self.steer = PIDNode()

    def close(self):
        self.running = False
        if self.sock:
            try: self.sock.shutdown(socket.SHUT_RDWR)
            except OSError: pass
        self.vision.close()
        self.wait(2500)

    def exact(self, n):
        buffer = bytearray()
        while len(buffer) < n and self.running:
            chunk = self.sock.recv(n - len(buffer))
            if not chunk: raise ConnectionError('camera closed')
            buffer.extend(chunk)
        if not self.running: raise ConnectionError('stopped')
        return bytes(buffer)

    def run(self):
        while self.running:
            try:
                self.sock = socket.create_connection((self.host, self.port), timeout=3)
                self.sock.settimeout(3)
                self.bus.status.emit('Camera connected to Raspberry Pi')
                while self.running:
                    n = struct.unpack('>L', self.exact(4))[0]
                    if not 1 <= n <= MAX_JPEG: raise ValueError('Invalid JPEG size')
                    frame = cv2.imdecode(np.frombuffer(self.exact(n), np.uint8), cv2.IMREAD_COLOR)
                    if frame is None: continue
                    with self.state.lock:
                        key = self.state.api_key
                    self.vision.set_api_key(key)
                    marked, data = self.vision.process(frame)
                    with self.state.lock:
                        allow = self.state.forward_allowed
                    decision = self.steer.decide(data, forward_allowed=allow)
                    self.state.update_vision(decision, data)
                    self.bus.frame_ready.emit(frame.copy(), marked.copy(), data)
                    self.bus.control_preview.emit(decision.label)
            except Exception as e:
                if self.running: self.bus.status.emit(f'Camera reconnecting: {e}')
                with self.state.lock:
                    self.state.confirmed = False
                    self.state.last_camera = 0.0
                    self.state.last_capture = 0.0
                    self.state.auto = False
            finally:
                if self.sock:
                    try: self.sock.close()
                    except OSError: pass
                self.sock = None
            if self.running: self.msleep(800)

class ControlWorker(threading.Thread):
    """Sends refresh commands at 10Hz; ESP32 watchdog stops on lost refresh."""
    def __init__(self, bus, state, host, port=5000):
        super().__init__(daemon=True)
        self.bus, self.state, self.host, self.port = bus, state, host, port
        self.running = True
        self.sock = None

    def close(self):
        self.running = False
        self.state.estop()
        if self.sock:
            try: self.sock.sendall(b'X')
            except OSError: pass
            try: self.sock.shutdown(socket.SHUT_RDWR)
            except OSError: pass
        self.join(timeout=2)

    def run(self):
        while self.running:
            sock = None
            try:
                sock = socket.create_connection((self.host, self.port), timeout=3)
                sock.settimeout(0.15)
                self.sock = sock
                with self.state.lock: self.state.connected = True
                self.bus.status.emit('Control TCP connected (disarmed)')
                last_send = 0
                while self.running:
                    now = time.monotonic()
                    if now - last_send >= 0.10:
                        cmd = self.state.command()
                        sock.sendall(cmd.encode('ascii'))
                        last_send = now
                    # Drain telemetry; avoid filling Pi socket buffer.
                    try: sock.recv(4096)
                    except socket.timeout: pass
                    time.sleep(0.01)
            except (OSError, ConnectionError) as e:
                if self.running: self.bus.status.emit(f'Control reconnecting: {e}')
            finally:
                with self.state.lock: self.state.connected = False
                self.state.estop()  # connection loss always disarms
                if sock:
                    try: sock.sendall(b'X')
                    except OSError: pass
                    try: sock.close()
                    except OSError: pass
                self.sock = None
            if self.running: time.sleep(1)
