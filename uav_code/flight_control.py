#!/usr/bin/env python3
import time
from pymavlink import mavutil

def set_mode_guided(m,f):
    modes = m.mode_mapping()
    if "GUIDED" not in modes:
        f.write("ERROR: GUIDED mode not supported.\n")
        return False

    guided_id = modes["GUIDED"]

    f.write("Setting mode GUIDED...\n")
    m.mav.set_mode_send(
        m.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        guided_id
    )

    # Chờ đến khi FCU báo vào GUIDED
    t0 = time.time()
    while time.time() - t0 < 8:
        hb = m.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        if hb and hb.custom_mode == guided_id:
            f.write("Mode = GUIDED\n")
            return True

    f.write("Timeout switching to GUIDED\n")
    return False

def arm(m,f):
    f.write("Arming motors...\n")
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0,0,0,0,0,0
    )

    t0 = time.time()
    while time.time() - t0 < 8:
        if m.motors_armed():
            f.write("Armed\n")
            return True
        msg = m.recv_match(type=["HEARTBEAT","STATUSTEXT"], blocking=True, timeout=1)
        if msg and msg.get_type()=="STATUSTEXT":
            f.write(f"[FCU] {getattr(msg, 'text')}\n")
    f.write("Arm timeout\n")
    return False

def takeoff(m, alt,f):
    f.write(f"Takeoff to {alt} m...")
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0,0,0,0,
        0,0,
        alt
    )

def land(m,f):
    f.write("Landing...")
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0,
        0,0,0,0,
        0,0,0
    )

def send_body_velocity(m, vx, vy, vz):

    type_mask = (
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_ACCELERATION_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
    )

    m.mav.set_position_target_local_ned_send(
        0,
        m.target_system,
        m.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        type_mask,
        0,0,0,
        vx, vy, vz,
        0,0,0,
        0,0
    )