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
ANNOUNCE_TIME = 2.0         # how long to keep announcing before waiting

# ============================================================


def pump(duration=3.0):
    """Call process_messages() for `duration` seconds to catch ACKs."""
    end = time.time() + duration
    while time.time() < end:
        # Heartbeat every iteration so the bridge keeps our address fresh
        # (and keeps the FC streaming telemetry to us) while we listen.
        xeroxuav.send_heartbeat()
        xeroxuav.process_messages()
        time.sleep(0.05)


def main():
    print(f"Connecting to bridge at {RPI_IP}:{UDP_PORT} ...")

    # Use 'udpout:' so pymavlink sends UDP packets to the Pi's bridge.
    # This matches the UDP socket bound by RPI_MAVLink_Bridge.py.
    # Do NOT use a raw TCP socket here — the bridge is UDP-only.
    #
    # wait=False: the bridge cannot forward anything until we send it a
    # packet, so announcing has to come before the heartbeat wait.
    master = xeroxuav.connect(f"udpout:{RPI_IP}:{UDP_PORT}", wait=False)

    # Announce ourselves first. The bridge learns our source address from
    # this heartbeat and only then starts forwarding FC telemetry to us.
    # Without this the bridge discards every message and we wait forever.
    print(f"Announcing to bridge for {ANNOUNCE_TIME}s ...")
    xeroxuav.announce(duration=ANNOUNCE_TIME)

    print("Waiting for heartbeat ...")
    print(f"Connected! System {master.target_system}, mode {master.flightmode}")

    print("\n[0] Setting GUIDED mode ...")
    xeroxuav.set_mode("GUIDED")
    pump(2.0)

    print("\n[1] Arming ...")
    xeroxuav.send_arm()
    pump(10.0)
    hb = master.wait_heartbeat(timeout=HEARTBEAT_TIMEOUT)
    if hb is None:
        print(f"\nNo heartbeat received within {HEARTBEAT_TIMEOUT}s. Aborting.")
        print("Checklist:")
        print(f"  1. Is the bridge running on the Pi? It should print "
              f"'Computer connected from ...' when we announce.")
        print(f"  2. Is {RPI_IP} still the Pi's IP? It changes when the "
              f"laptop moves to another network.")
        print("  3. Is anything else bound to UDP 14550 on the Pi?")
        print("  4. Is a firewall dropping UDP between these two hosts?")
        return

    print(f"Connected! System {master.target_system}, mode {master.flightmode}")

    print("\n[1] Arming ...")
    xeroxuav.send_arm()
    pump(10.0)

    print("\n[2] Velocity commands (only effective while armed + airborne):")
    xeroxuav.send_velocity(0.0, 0.0, 0.0)      # hold position
    time.sleep(1.0)
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