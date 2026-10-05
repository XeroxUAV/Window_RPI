#!/usr/bin/env python3
"""
Test navigation commands over the network using the xeroxuav library.

Sequence (safe bench test, no load):
    connect -> arm -> velocity self-test -> stop velocity -> disarm

WARNING: removing the propellers is strongly recommended for a bench test.
"""

import time
import xeroxuav

# ============================================================
# CONFIGURATION
# ============================================================

RPI_IP = '10.178.125.230'   # <-- change to your Pi's IP address
UDP_PORT = 14550
HEARTBEAT_TIMEOUT = 10      # options: higher with camera stream running

# ============================================================


def pump(duration=3.0):
    """Call process_messages() for `duration` seconds to catch ACKs."""
    end = time.time() + duration
    while time.time() < end:
        xeroxuav.process_messages()
        time.sleep(0.05)


def main():
    print(f"Connecting to bridge at {RPI_IP}:{UDP_PORT} ...")

    # Use 'udpout:' so pymavlink sends UDP packets to the Pi's bridge.
    # This matches the UDP socket bound by RPI_MAVLink_Bridge.py.
    # Do NOT use a raw TCP socket here — the bridge is UDP-only.
    master = xeroxuav.connect(f"udpout:{RPI_IP}:{UDP_PORT}")

    # Wait for a heartbeat so we know the link is really up before
    # sending commands. Without this, send_arm() may be sent into the void.
    print("Waiting for heartbeat ...")
    hb = master.wait_heartbeat(timeout=HEARTBEAT_TIMEOUT)
    if hb is None:
        print(f"No heartbeat received within {HEARTBEAT_TIMEOUT}s. Aborting.")
        return

    print(f"Connected! System {master.target_system}, mode {master.flightmode}")

    print("\n[1] Arming ...")
    xeroxuav.send_arm()
    pump(3.0)

    print("\n[2] Velocity commands (only effective while armed + airborne):")
    # xeroxuav.send_velocity(0.0, 0.0, 0.0)      # hold position
    # time.sleep(1.0)
    # xeroxuav.send_velocity(0.2, 0.0, 0.0)      # 0.2 m/s to the right, 2 s
    # time.sleep(2.0)
    # xeroxuav.send_velocity(0.0, 0.3, 0.0)      # 0.3 m/s forward, 2 s
    # time.sleep(2.0)
    # xeroxuav.send_velocity(0.0, 0.0, 0.0, yaw_rate=30.0)  # yaw right 30 deg/s
    # time.sleep(2.0)
    # xeroxuav.send_velocity(0.0, 0.0, 0.0)      # stop
    # pump(1.0)

    print("\n[3] Disarming ...")
    xeroxuav.send_disarm()
    pump(3.0)

    print("\nDone. Command round-trip test complete.")


if __name__ == '__main__':
    main()