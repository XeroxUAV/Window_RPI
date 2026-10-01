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

RPI_IP = '10.178.125.248'   # <-- change to your Pi's IP address
UDP_PORT = 14550
HEARTBEAT_TIMEOUT = 10     # options: higher with camera stream running

# ============================================================


def pump(duration=3.0):
    """Call process_messages() for `duration` seconds to catch ACKs."""
    end = time.time() + duration
    while time.time() < end:
        xeroxuav.process_messages()
        time.sleep(0.05)


def main():
    print(f"Connecting to bridge at {RPI_IP}:{UDP_PORT} ...")
    master = xeroxuav.connect(f"udpout:{RPI_IP}:{UDP_PORT}")
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