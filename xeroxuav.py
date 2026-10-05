"""
xeroxuav.py — MAVLink control library for the XeroxUAV drone.

Functions:
    connect()          - open a MAVLink connection
    announce()         - send GCS heartbeats so the RPi bridge finds us
    send_heartbeat()   - send a single GCS heartbeat
    send_velocity()    - send a velocity + yaw-rate command (GUIDED mode)
    ekf_source()       - switch the active EKF source set
    send_land()        - command the vehicle to land
    send_takeoff()     - command the vehicle to take off
    send_arm()         - arm the vehicle
    process_messages() - drain and handle incoming MAVLink messages
"""

import time

from pymavlink import mavutil

# ============================================================
# Constants
# ============================================================

MAX_VELOCITY = 0.5      # m/s — max magnitude for right / forward / up
MAX_YAW_RATE = 200.0    # deg/s — max magnitude for yaw rate
MAV_CMD_SET_EKF_SOURCE_SET = 42007

# Type masks for SET_POSITION_TARGET_LOCAL_NED.
# Bit 1 = ignore, Bit 0 = use.
#   3527 = 0b110111000111 → ignore pos, use vel, ignore accel, ignore yaw & yaw_rate
#   1479 = 0b010111000111 → ignore pos, use vel, ignore accel, ignore yaw, USE yaw_rate
TYPE_MASK_VELOCITY_ONLY         = 3527
TYPE_MASK_VELOCITY_AND_YAW_RATE = 1479


# ============================================================
# Internal state
# ============================================================

_master = None
_pending = {}


# ============================================================
# Connection
# ============================================================

def connect(connection_string="udp:0.0.0.0:14552",
            source_system=255,
            source_component=190,
            wait=True,
            timeout=5.0):
    """
    Open a MAVLink connection and (optionally) wait for the first heartbeat.

    Parameters
    ----------
    connection_string : str
        e.g. "udpout:10.0.0.5:14550" or "/dev/ttyAMA4"
    source_system : int
        MAVLink system ID for this GCS (default 255)
    source_component : int
        MAVLink component ID for this GCS (default 190)
    wait : bool
        If True, block for up to `timeout` seconds waiting for a heartbeat.
        Pass wait=False to control the handshake yourself (e.g. call
        announce() first) — otherwise you silently wait twice and the
        caller's own timeout stops reflecting reality.
    timeout : float
        Seconds to wait for the first heartbeat when `wait` is True.
    """
    global _master
    _master = mavutil.mavlink_connection(
        connection_string,
        source_system=source_system,
        source_component=source_component,
    )
    if wait:
        _master.wait_heartbeat(timeout=timeout)
    return _master


# ============================================================
# Handshake
# ============================================================

def send_heartbeat():
    """
    Send a single GCS heartbeat.

    This is what makes the RPi bridge aware of us. The bridge is
    reactive-only: it learns the computer's source address from the first
    packet it receives and discards everything until then. So if we wait
    for telemetry before speaking, the bridge has nowhere to send it and
    the link deadlocks.
    """
    if _master is None:
        raise RuntimeError("Not connected. Call xeroxuav.connect() first.")
    _master.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_GCS,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        0, 0, 0,
    )


def announce(duration=2.0, interval=0.2):
    """
    Send GCS heartbeats for `duration` seconds so the bridge learns our
    address. Call this immediately after connect() with wait=False.

    Several are sent (rather than one) so a single dropped packet on a
    busy WiFi link cannot leave us stuck waiting again.
    """
    end = time.time() + duration
    while True:
        send_heartbeat()
        if time.time() + interval >= end:
            return
        time.sleep(interval)


# ============================================================
# Helpers
# ============================================================

def _clamp(value, limit=MAX_VELOCITY):
    """Clamp a value to the range [-limit, +limit]."""
    return max(-limit, min(limit, value))


def process_messages():
    """
    Drain incoming MAVLink messages and handle COMMAND_ACK.

    Call this periodically from your main loop to print
    ACCEPTED / REJECTED for any command you sent.
    """
    if _master is None:
        return
    while True:
        msg = _master.recv_match(blocking=False)
        if msg is None:
            break
        if msg.get_type() == "COMMAND_ACK":
            cmd = msg.command
            if cmd in _pending:
                name = _pending.pop(cmd)
                if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                    print(f"{name} ACCEPTED")
                else:
                    print(f"{name} REJECTED (result={msg.result})")


# ============================================================
# Public API
# ============================================================

def send_velocity(right, forward, up, yaw_rate=0.0):
    """
    Send a velocity + yaw-rate command in the BODY_OFFSET_NED frame.

    Parameters
    ----------
    right : float
        Velocity to the right in m/s.
        + = right, − = left. Clamped to ±MAX_VELOCITY (0.5).
    forward : float
        Velocity forward in m/s.
        + = forward, − = backward. Clamped to ±MAX_VELOCITY.
    up : float
        Velocity upward in m/s.
        + = up, − = down. Clamped to ±MAX_VELOCITY.
    yaw_rate : float, optional
        Desired yaw rate in degrees per second.
        + = clockwise, − = counter-clockwise.
        Clamped to ±MAX_YAW_RATE (200 deg/s). Default 0.

    Notes
    -----
    * ArduPilot's NED frame has +Z pointing DOWN, so an upward
      command is sent as a negative Z velocity.
    * MAVLink expects yaw_rate in radians/second, so the input
      (deg/s) is converted automatically.
    * The achievable yaw rate is also limited by the flight
      controller's PILOT_Y_RATE parameter (default 200 deg/s).
    """
    if _master is None:
        raise RuntimeError("Not connected. Call xeroxuav.connect() first.")

    # Convert user inputs (right / forward / up) to ArduPilot NED
    vy = _clamp(right)
    vx = _clamp(forward)
    vz = -_clamp(up)                # NED +Z = down → up is negative Z

    # Clamp yaw rate and convert deg/s → rad/s
    yaw_rate_clamped = max(-MAX_YAW_RATE, min(MAX_YAW_RATE, yaw_rate))
    yaw_rate_rad = yaw_rate_clamped * 3.141592653589793 / 180.0

    _master.mav.set_position_target_local_ned_send(
        0,                                       # time_boot_ms
        _master.target_system,
        _master.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        TYPE_MASK_VELOCITY_AND_YAW_RATE,
        0, 0, 0,                                 # position (ignored)
        vx, vy, vz,                              # velocity
        0, 0, 0,                                 # acceleration (ignored)
        0, yaw_rate_rad,                         # yaw (ignored), yaw_rate
    )


def ekf_source(source_set):
    """
    Switch the active EKF source set.

    Parameters
    ----------
    source_set : int
        1, 2, or 3 — which EK3_SRCx set to activate.

    The actual sensors assigned to each set are configured
    in the flight controller via the EK3_SRC1_*, EK3_SRC2_*,
    and EK3_SRC3_* parameters.
    """
    if _master is None:
        raise RuntimeError("Not connected. Call xeroxuav.connect() first.")
    if source_set not in (1, 2, 3):
        raise ValueError(f"source_set must be 1, 2 or 3 (got {source_set})")

    _master.mav.command_long_send(
        _master.target_system,
        _master.target_component,
        MAV_CMD_SET_EKF_SOURCE_SET,
        0,
        source_set,
        0, 0, 0, 0, 0, 0,
    )
    _pending[MAV_CMD_SET_EKF_SOURCE_SET] = f"EKF source set {source_set}"


def send_land():
    """Command the vehicle to land at its current position."""
    if _master is None:
        raise RuntimeError("Not connected. Call xeroxuav.connect() first.")
    _master.mav.command_long_send(
        _master.target_system,
        _master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0,
        0, 0, 0, 0, 0, 0, 0,
    )
    _pending[mavutil.mavlink.MAV_CMD_NAV_LAND] = "Land"


def send_takeoff(altitude):
    """
    Command the vehicle to take off.

    Parameters
    ----------
    altitude : float
        Target altitude in metres, relative to home.
    """
    if _master is None:
        raise RuntimeError("Not connected. Call xeroxuav.connect() first.")
    _master.mav.command_long_send(
        _master.target_system,
        _master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, 0, 0, 0,
        altitude,
    )
    _pending[mavutil.mavlink.MAV_CMD_NAV_TAKEOFF] = "Takeoff"


def send_arm():
    """
    Arm the vehicle.

    Note: the physical RC arm switch must be in the ARM position,
    otherwise the flight controller will reject the command.
    """
    if _master is None:
        raise RuntimeError("Not connected. Call xeroxuav.connect() first.")
    _master.mav.command_long_send(
        _master.target_system,
        _master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0,
    )
    _pending[mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM] = "Arm"