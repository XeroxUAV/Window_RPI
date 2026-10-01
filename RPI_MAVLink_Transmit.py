#!/usr/bin/env python3
"""
Arm the vehicle via MAVLink over Raspberry Pi UART4.
Device: /dev/ttyAMA4
Baud: 115200
"""

from pymavlink import mavutil
import time

# ============================================================
# CONFIGURATION
# ============================================================

SERIAL_PORT = '/dev/ttyAMA4'
BAUD_RATE = 115200

# ============================================================
# CONNECT
# ============================================================

print(f"Connecting on {SERIAL_PORT} at {BAUD_RATE} baud...")

master = mavutil.mavlink_connection(
    SERIAL_PORT,
    baud=BAUD_RATE
)

print("Waiting for heartbeat...")
master.wait_heartbeat()

print("Connected!")
print(f"System ID: {master.target_system}")
print(f"Component ID: {master.target_component}")
print(f"Flight mode: {master.flightmode}")

# ============================================================
# ARM COMMAND
# ============================================================

print("\nSending ARM command...")

master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1,      # param1: 1 = arm
    0,      # param2: 0 = normal (21196 = force)
    0, 0, 0, 0, 0
)

# ============================================================
# WAIT FOR ACK
# ============================================================

ack = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)

if ack and ack.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
    if ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
        print("Arm ACCEPTED")
    else:
        print(f"Arm REJECTED (result: {ack.result})")
else:
    print("⚠️ No acknowledgment received")

master.close()
print("Done.")