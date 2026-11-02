"""Pi 5 server: /dev/video0 EMEET, TCP JPEG on 5001, ESP32 bridge on 5000.
IMPORTANT: Requires updated ESP32 firmware with its own 500ms motor watchdog.
"""
import cv2
import serial
import socket
import struct
import threading
import time
import os
import signal

HOST = '0.0.0.0'
CMD_PORT = 5000
CAM_PORT = 5001
CAMERA_DEVICE = '/dev/video0'
SERIAL_PORT = '/dev/ttyACM0'
BAUD = 115200
JPEG_QUALITY = 70
STOP = threading.Event()
SERIAL_LOCK = threading.Lock()
SER = None
VALID = set(b'WASDXP')


def write_esp(payload):
    with SERIAL_LOCK:
        if SER is None or not SER.is_open: return False
        try:
            SER.write(payload)
            SER.flush()
            return True
        except (serial.SerialException, OSError):
            return False


def connect_serial():
    global SER
    while not STOP.is_set():
        with SERIAL_LOCK:
            if SER is not None and SER.is_open:
                pass
            else:
                try:
                    SER = serial.Serial(SERIAL_PORT, BAUD, timeout=0.06, write_timeout=0.3)
                    print('[ESP32] connected', SERIAL_PORT, flush=True)
                    # ESP32 reset on USB open; its startup state must be stopped.
                except Exception as e:
                    SER = None
                    print('[ESP32] waiting:', e, flush=True)
        STOP.wait(0.5 if SER is None else 0.2)


def control_server():
    global SER
    server = socket.socket()
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((HOST, CMD_PORT))
    server.listen(1)
    server.settimeout(1)
    print('[Control] port', CMD_PORT, flush=True)
    try:
        while not STOP.is_set():
            try: client, addr = server.accept()
            except socket.timeout: continue
            print('[Control] client', addr, flush=True)
            client.settimeout(0.08)
            # Never allow stale client commands to continue after link loss.
            last_command = time.monotonic()
            try:
                while not STOP.is_set():
                    try:
                        payload = client.recv(256)
                        if not payload: break
                        last_command = time.monotonic()
                        for c in payload:
                            if c in VALID: write_esp(bytes([c]))
                    except socket.timeout: pass
                    # Best effort stop on TCP silence, independently of ESP32 watchdog.
                    if time.monotonic() - last_command > 0.3:
                        write_esp(b'X')
                    with SERIAL_LOCK:
                        if SER and SER.is_open:
                            try: line = SER.readline()
                            except (OSError, serial.SerialException):
                                try: SER.close()
                                except Exception: pass
                                SER = None
                                line = b''
                        else: line = b''
                    if line:
                        try: client.sendall(line)
                        except (OSError, ConnectionError): break
            except (OSError, ConnectionError) as e:
                print('[Control] disconnected:', e, flush=True)
            finally:
                write_esp(b'X')
                client.close()
                print('[Control] stopped', flush=True)
    finally: server.close()


def camera_server():
    server = socket.socket()
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((HOST, CAM_PORT))
    server.listen(1)
    server.settimeout(1)
    print('[Camera] port', CAM_PORT, flush=True)
    try:
        while not STOP.is_set():
            try: client, addr = server.accept()
            except socket.timeout: continue
            print('[Camera] client', addr, flush=True)
            cam = cv2.VideoCapture(CAMERA_DEVICE, cv2.CAP_V4L2)
            try:
                if not cam.isOpened():
                    print('[Camera] cannot open', CAMERA_DEVICE, flush=True)
                    continue
                cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
                cam.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                cam.set(cv2.CAP_PROP_FPS, 30)
                failures = 0
                while not STOP.is_set():
                    ok, frame = cam.read()
                    if not ok:
                        failures += 1
                        if failures >= 8: break
                        time.sleep(0.1)
                        continue
                    failures = 0
                    ok, jpg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
                    if not ok: continue
                    data = jpg.tobytes()
                    client.sendall(struct.pack('>L', len(data)) + data)
                    time.sleep(0.025)
            except (OSError, ConnectionError) as e:
                print('[Camera] disconnected:', e, flush=True)
            finally:
                cam.release()
                client.close()
    finally: server.close()


def end(_sig, _frame):
    STOP.set()
    write_esp(b'X')


def main():
    signal.signal(signal.SIGINT, end)
    signal.signal(signal.SIGTERM, end)
    print('Pi 5 EMEET C950: TCP 5000 commands/RPM and 5001 camera', flush=True)
    threading.Thread(target=connect_serial, daemon=True).start()
    threading.Thread(target=control_server, daemon=True).start()
    threading.Thread(target=camera_server, daemon=True).start()
    while not STOP.wait(0.2): pass
    write_esp(b'X')
    with SERIAL_LOCK:
        if SER:
            try: SER.close()
            except Exception: pass

if __name__ == '__main__': main()
