#!/usr/bin/env python3
"""
Bidirectional MAVLink bridge: Flight Controller <-> Computer.
RPi side: connects to the flight controller over UART and forwards
all MAVLink traffic to/from your computer over UDP.
"""

import socket
import threading
from pymavlink import mavutil

# ============================================================
# CONFIGURATION
# ============================================================

SERIAL_PORT = '/dev/ttyAMA4'
BAUD_RATE = 115200
UDP_IP = '0.0.0.0'
UDP_PORT = 14550

# ============================================================

client_addr = None
addr_lock = threading.Lock()


def serial_to_udp(serial, udp_sock):
    while True:
        n = serial.port.in_waiting or 512
        data = serial.port.read(n)
        if data:
            with addr_lock:
                addr = client_addr
            if addr is not None:
                udp_sock.sendto(data, addr)


def udp_to_serial(udp_sock, serial):
    global client_addr
    while True:
        data, addr = udp_sock.recvfrom(4096)
        with addr_lock:
            if client_addr != addr:
                old = client_addr
                client_addr = addr
                if old is None:
                    print(f"Computer connected from {addr[0]}:{addr[1]}")
                else:
                    print(f"Telemetry now going to {addr[0]}:{addr[1]}")
        serial.write(data)


def main():
    print(f"Connecting to flight controller on {SERIAL_PORT} at {BAUD_RATE} baud...")
    serial = mavutil.mavlink_connection(SERIAL_PORT, baud=BAUD_RATE)
    serial.wait_heartbeat()
    print(f"Flight controller connected (system {serial.target_system}).")

    udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_sock.bind((UDP_IP, UDP_PORT))
    print(f"UDP bridge listening on {UDP_IP}:{UDP_PORT}...")
    print("Now run the PC script to connect.")

    t1 = threading.Thread(target=serial_to_udp, args=(serial, udp_sock), daemon=True)
    t2 = threading.Thread(target=udp_to_serial, args=(udp_sock, serial), daemon=True)
    t1.start()
    t2.start()
    t1.join()
    t2.join()


if __name__ == '__main__':
    main()