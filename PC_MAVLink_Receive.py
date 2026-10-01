#!/usr/bin/env python3
"""
MAVLink client for the computer side.
Connects over UDP to the RPi bridge and prints telemetry from the flight controller.
"""

from pymavlink import mavutil
import time

# ============================================================
# CONFIGURATION
# ============================================================

RPI_IP = '10.162.37.248'   # <-- change to your Pi's IP address
UDP_PORT = 14550

# ============================================================


def main():
    print(f"Connecting to bridge at {RPI_IP}:{UDP_PORT} ...")
    master = mavutil.mavlink_connection(f"udpout:{RPI_IP}:{UDP_PORT}")

    # Wake up the bridge so it learns our address on the network
    master.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_GCS,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        0, 0, 0
    )

    print("Waiting for heartbeat from flight controller...")
    master.wait_heartbeat()
    print(f"Connected! System {master.target_system}, component {master.target_component}, "
          f"mode {master.flightmode}")

    print("Printing telemetry. Press Ctrl+C to stop.\n")
    while True:
        msg = master.recv_match(
            type=['HEARTBEAT', 'SYS_STATUS', 'ATTITUDE', 'GPS_RAW_INT', 'VFR_HUD'],
            blocking=True, timeout=1
        )
        if msg is None:
            continue

        msg_type = msg.get_type()
        if msg_type == 'HEARTBEAT':
            print(f"Heartbeat: mode={master.flightmode}")
        elif msg_type == 'SYS_STATUS':
            voltage = msg.voltage_battery / 1000.0
            current = msg.current_battery / 100.0
            print(f"Battery: {voltage:.2f} V | {current:.1f} A")
        elif msg_type == 'ATTITUDE':
            print(f"Attitude: roll {msg.roll:.2f} | pitch {msg.pitch:.2f} | yaw {msg.yaw:.2f}")
        elif msg_type == 'GPS_RAW_INT':
            print(f"GPS: {msg.lat / 1e7:.6f}, {msg.lon / 1e7:.6f} "
                  f"| fix {msg.fix_type} | sats {msg.satellites_visible}")
        elif msg_type == 'VFR_HUD':
            print(f"Speed {msg.groundspeed:.1f} m/s | alt {msg.alt:.1f} m | heading {msg.heading}")


if __name__ == '__main__':
    main()