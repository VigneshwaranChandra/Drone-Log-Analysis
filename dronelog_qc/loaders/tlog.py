"""MAVLink telemetry log (.tlog) loader.

A .tlog is what Mission Planner / QGroundControl style GCS software records:
every MAVLink packet received over the radio, each prefixed with an 8-byte
big-endian wall-clock timestamp (microseconds). Telemetry rates are usually
low (2-50 Hz), which limits oscillation analysis - the report says so.

Time base: we use the autopilot's own boot clock (time_boot_ms / time_usec)
where a message has one, because GCS receive time jitters with the radio.
Messages without a boot clock are mapped onto it using a running offset
between wall clock and boot clock.
"""
from __future__ import annotations

import math
from collections import defaultdict

import numpy as np

from ..data import Event, FlightData

# MAV_AUTOPILOT enum -> name (subset, for the fingerprint section)
AUTOPILOT_NAMES = {
    0: "GENERIC", 3: "ARDUPILOTMEGA", 4: "OPENPILOT", 8: "INVALID (GCS/other)",
    12: "PX4", 13: "SMACCMPILOT", 14: "AUTOQUAD", 15: "ARMAZILA", 16: "AEROB",
    17: "ASLUAV", 18: "SMARTAP", 19: "AIRRAILS", 20: "REFLEX",
}
MAV_TYPE_NAMES = {1: "FIXED_WING", 2: "QUADROTOR", 3: "COAXIAL", 4: "HELICOPTER",
                  6: "GCS", 13: "HEXAROTOR", 14: "OCTOROTOR", 15: "TRICOPTER",
                  19: "VTOL_TAILSITTER_DUOROTOR", 20: "VTOL_QUAD", 21: "VTOL_TILTROTOR"}


def _quat_to_euler(w, x, y, z):
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    s = max(-1.0, min(1.0, 2 * (w * y - z * x)))
    pitch = math.asin(s)
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def load_tlog(path: str) -> FlightData:
    from pymavlink import mavutil  # imported lazily so CSV-only users don't need it

    mlog = mavutil.mavlink_connection(path, dialect="ardupilotmega", robust_parsing=True)
    fd = FlightData(source=path)
    cols = defaultdict(lambda: ([], []))   # name -> (t list, v list)
    msg_counts = defaultdict(int)
    sysids = defaultdict(int)
    texts = []
    heartbeat_info = {}
    autopilot_version = None
    last_mode = None
    last_armed = None
    clock_offset = None   # wall - boot   (seconds)
    home = None

    def put(name, t, v):
        c = cols[name]
        c[0].append(t)
        c[1].append(v)

    while True:
        try:
            m = mlog.recv_match(blocking=False)
        except Exception:
            continue
        if m is None:
            break
        mtype = m.get_type()
        if mtype == "BAD_DATA":
            msg_counts["BAD_DATA"] += 1
            continue
        msg_counts[mtype] += 1
        src = m.get_srcSystem()
        sysids[src] += 1
        wall = getattr(m, "_timestamp", None)

        # ---- time on boot clock
        tb = None
        if hasattr(m, "time_boot_ms"):
            tb = m.time_boot_ms / 1000.0
        elif mtype == "SERVO_OUTPUT_RAW":
            # time_usec here is uint32 and wraps every ~71.6 min -> use wall clock
            tb = None
        elif hasattr(m, "time_usec") and mtype not in ("SYSTEM_TIME", "GPS_RAW_INT", "GPS2_RAW"):
            tb = m.time_usec / 1e6
        if tb is not None and wall is not None and tb > 0:
            off = wall - tb
            clock_offset = off if clock_offset is None else 0.98 * clock_offset + 0.02 * off
        if tb is None or tb <= 0:
            if wall is not None and clock_offset is not None:
                tb = wall - clock_offset
            elif mtype in ("HEARTBEAT", "STATUSTEXT", "PARAM_VALUE", "AUTOPILOT_VERSION"):
                tb = 0.0          # before the first boot-timed packet
            else:
                continue

        # ---- per-message mapping
        if mtype == "HEARTBEAT":
            if m.type == 6 or m.autopilot == 8:     # GCS heartbeat, ignore
                continue
            heartbeat_info = {"autopilot": int(m.autopilot), "type": int(m.type),
                              "sysid": src, "mavlink_version": int(m.mavlink_version)}
            armed = bool(m.base_mode & 128)
            try:
                mode = mavutil.mode_string_v10(m)
            except Exception:
                mode = f"custom_mode={m.custom_mode}"
            if mode != last_mode:
                fd.events.append(Event(tb, "mode", f"Mode -> {mode}"))
                last_mode = mode
            if armed != last_armed:
                fd.events.append(Event(tb, "arm" if armed else "disarm",
                                       "ARMED" if armed else "DISARMED"))
                last_armed = armed
            put("armed", tb, 1.0 if armed else 0.0)
        elif mtype == "ATTITUDE":
            put("roll", tb, math.degrees(m.roll))
            put("pitch", tb, math.degrees(m.pitch))
            put("yaw", tb, math.degrees(m.yaw) % 360.0)
            put("p", tb, math.degrees(m.rollspeed))
            put("q", tb, math.degrees(m.pitchspeed))
            put("r", tb, math.degrees(m.yawspeed))
        elif mtype == "ATTITUDE_QUATERNION" and msg_counts["ATTITUDE"] == 0:
            r_, p_, y_ = _quat_to_euler(m.q1, m.q2, m.q3, m.q4)
            put("roll", tb, r_); put("pitch", tb, p_); put("yaw", tb, y_ % 360)
        elif mtype == "ATTITUDE_TARGET":
            q = m.q
            if any(abs(x) > 1e-6 for x in q):
                r_, p_, y_ = _quat_to_euler(*q)
                put("roll_sp", tb, r_); put("pitch_sp", tb, p_); put("yaw_sp", tb, y_ % 360)
            put("p_sp", tb, math.degrees(m.body_roll_rate))
            put("q_sp", tb, math.degrees(m.body_pitch_rate))
            put("r_sp", tb, math.degrees(m.body_yaw_rate))
            put("thrust_sp", tb, m.thrust * 100.0)
        elif mtype == "NAV_CONTROLLER_OUTPUT":
            put("nav_roll", tb, m.nav_roll)
            put("nav_pitch", tb, m.nav_pitch)
            put("nav_bearing", tb, m.nav_bearing)
            put("wp_dist", tb, m.wp_dist)
        elif mtype == "SERVO_OUTPUT_RAW":
            port = getattr(m, "port", 0)
            for i in range(1, 17):
                v = getattr(m, f"servo{i}_raw", None)
                if v is None:
                    continue
                put(f"servo_{port * 16 + i}", tb, v)
        elif mtype == "ACTUATOR_OUTPUT_STATUS":
            for i, v in enumerate(m.actuator[: m.active], start=1):
                put(f"servo_{i}", tb, v)
        elif mtype == "GLOBAL_POSITION_INT":
            lat, lon = m.lat / 1e7, m.lon / 1e7
            if lat != 0 or lon != 0:
                put("lat", tb, lat); put("lon", tb, lon)
                if home is None:
                    home = (lat, lon)
            put("alt_rel", tb, m.relative_alt / 1000.0)
            put("alt_msl", tb, m.alt / 1000.0)
            put("vel_n", tb, m.vx / 100.0)
            put("vel_e", tb, m.vy / 100.0)
            put("vel_u", tb, -m.vz / 100.0)
        elif mtype == "LOCAL_POSITION_NED":
            put("lpos_n", tb, m.x); put("lpos_e", tb, m.y); put("lpos_u", tb, -m.z)
            put("lvel_n", tb, m.vx); put("lvel_e", tb, m.vy); put("lvel_u", tb, -m.vz)
        elif mtype == "GPS_RAW_INT":
            put("gps_sats", tb, m.satellites_visible)
            put("gps_fix", tb, m.fix_type)
            if m.eph not in (0, 65535):
                put("gps_hdop", tb, m.eph / 100.0)
        elif mtype == "GPS2_RAW":
            put("gps2_sats", tb, m.satellites_visible)
            put("gps2_fix", tb, m.fix_type)
        elif mtype == "SYS_STATUS":
            if m.voltage_battery not in (0, 65535):
                put("volt", tb, m.voltage_battery / 1000.0)
            if m.current_battery >= 0:
                put("curr", tb, m.current_battery / 100.0)
            if m.battery_remaining >= 0:
                put("batt_pct", tb, m.battery_remaining)
        elif mtype == "BATTERY_STATUS" and msg_counts["SYS_STATUS"] == 0:
            cells = [c for c in m.voltages if c not in (65535, 0)]
            if cells:
                put("volt", tb, sum(cells) / 1000.0)
            if m.current_battery >= 0:
                put("curr", tb, m.current_battery / 100.0)
        elif mtype == "VFR_HUD":
            put("groundspeed", tb, m.groundspeed)
            put("airspeed", tb, m.airspeed)
            put("throttle", tb, m.throttle)
            put("climb", tb, m.climb)
        elif mtype in ("RAW_IMU", "SCALED_IMU", "SCALED_IMU2", "HIGHRES_IMU"):
            if mtype == "SCALED_IMU2" and msg_counts["SCALED_IMU"] + msg_counts["RAW_IMU"] > 0:
                continue
            k = 9.80665 / 1000.0 if mtype != "HIGHRES_IMU" else 1.0
            put("acc_x", tb, m.xacc * k); put("acc_y", tb, m.yacc * k); put("acc_z", tb, m.zacc * k)
        elif mtype == "VIBRATION":
            put("vibe_x", tb, m.vibration_x); put("vibe_y", tb, m.vibration_y)
            put("vibe_z", tb, m.vibration_z)
            put("clip", tb, m.clipping_0 + m.clipping_1 + m.clipping_2)
        elif mtype == "WIND":
            put("wind_fc_dir", tb, m.direction); put("wind_fc_speed", tb, m.speed)
        elif mtype == "RC_CHANNELS":
            for i in range(1, 9):
                put(f"rc_{i}", tb, getattr(m, f"chan{i}_raw"))
        elif mtype == "STATUSTEXT":
            txt = m.text if isinstance(m.text, str) else m.text.decode("utf-8", "replace")
            texts.append((tb, int(m.severity), txt))
            kind = "failsafe" if "fail" in txt.lower() else "text"
            fd.events.append(Event(tb, kind, txt.strip("\x00 "), int(m.severity)))
        elif mtype == "PARAM_VALUE":
            pid = m.param_id if isinstance(m.param_id, str) else m.param_id.decode("ascii", "replace")
            fd.params[pid.strip("\x00")] = float(m.param_value)
        elif mtype == "AUTOPILOT_VERSION":
            autopilot_version = {
                "flight_sw_version": hex(m.flight_sw_version),
                "middleware_sw_version": hex(m.middleware_sw_version),
                "os_sw_version": hex(m.os_sw_version),
                "board_version": hex(m.board_version),
                "vendor_id": int(m.vendor_id), "product_id": int(m.product_id),
                "uid": hex(m.uid),
                "flight_custom_version": bytes(m.flight_custom_version).hex(),
            }

    for name, (t, v) in cols.items():
        fd.add(name, t, v)

    _pick_motors(fd)
    _derive_position(fd, home)
    if "lpos_n" in fd.series and "pos_n" not in fd.series:
        for a in ("n", "e", "u"):
            fd.series[f"pos_{a}"] = fd.series[f"lpos_{a}"]
    for a in ("n", "e", "u"):
        if f"vel_{a}" not in fd.series and f"lvel_{a}" in fd.series:
            fd.series[f"vel_{a}"] = fd.series[f"lvel_{a}"]
    if "roll_sp" not in fd.series and "nav_roll" in fd.series:
        fd.series["roll_sp"] = fd.series["nav_roll"]
        fd.series["pitch_sp"] = fd.series["nav_pitch"]
        fd.meta["setpoint_source"] = "NAV_CONTROLLER_OUTPUT (nav_roll/nav_pitch)"
    elif "roll_sp" in fd.series:
        fd.meta["setpoint_source"] = "ATTITUDE_TARGET"

    hb = heartbeat_info
    fd.meta.update({
        "format": "MAVLink tlog",
        "message_counts": dict(sorted(msg_counts.items(), key=lambda kv: -kv[1])),
        "sysids": dict(sysids),
        "autopilot_enum": hb.get("autopilot"),
        "autopilot_name": AUTOPILOT_NAMES.get(hb.get("autopilot"), str(hb.get("autopilot"))),
        "vehicle_type": MAV_TYPE_NAMES.get(hb.get("type"), str(hb.get("type"))),
        "autopilot_version": autopilot_version,
        "statustext": texts[:300],
        "home": home,
    })
    # Put t=0 at the start of the log (boot clock value kept in meta)
    if fd.series:
        t0, _ = fd.time_range()
        fd.meta["log_start_boot_s"] = t0
        fd.shift_time(t0)
    return fd


def _pick_motors(fd: FlightData) -> None:
    """Servo channels that actually move like motors become motor_1..N."""
    servo = [n for n in fd.series if n.startswith("servo_")]
    servo.sort(key=lambda s: int(s.split("_")[1]))
    motors = []
    for n in servo:
        _, v = fd.series[n]
        if v.size < 10:
            continue
        lo, hi = np.percentile(v, 2), np.percentile(v, 98)
        if hi > 900 and np.max(v) <= 2200 and (hi - lo) > 3:
            motors.append(n)
    # Multirotor motors are normally the first contiguous outputs (4/6/8)
    if len(motors) >= 4:
        nums = [int(n.split("_")[1]) for n in motors]
        first = nums[0]
        contiguous = [n for n, k in zip(motors, nums) if k - first == nums.index(k)]
        motors = contiguous if len(contiguous) >= 4 else motors
    for i, n in enumerate(motors, start=1):
        fd.series[f"motor_{i}"] = fd.series[n]
    fd.meta["motor_channels"] = motors
    fd.meta["motor_units"] = "PWM_us"


def _derive_position(fd: FlightData, home) -> None:
    if home is None or not fd.has("lat", "lon"):
        return
    t, lat = fd.get("lat")
    _, lon = fd.get("lon")
    r = 6378137.0
    pn = np.radians(lat - home[0]) * r
    pe = np.radians(lon - home[1]) * r * math.cos(math.radians(home[0]))
    fd.add("pos_n", t, pn)
    fd.add("pos_e", t, pe)
    if fd.has("alt_rel"):
        fd.series["pos_u"] = fd.series["alt_rel"]
