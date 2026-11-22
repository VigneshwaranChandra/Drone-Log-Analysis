"""Quadcopter flight simulator that writes RFly-format .rlog files, with fault injection.

Used to prove that every detector in the analysis really fires on the fault it is
meant for (and stays quiet on a healthy flight).

    python tools/simulate_rlog.py out.rlog --fault cg_offset
    python tools/simulate_rlog.py --list

Physical layout simulated = config default: M1 front-right, M2 front-left,
M3 rear-left, M4 rear-right (FRD body, NED world).
"""
from __future__ import annotations

import argparse
import math
import struct

import numpy as np

FAULTS = {
    "none": "healthy flight",
    "cg_offset": "battery shifted forward-right (constant roll+pitch disturbance)",
    "weak_motor": "M3 produces 82% thrust (worn motor / chipped prop)",
    "motor_tilt": "motors twisted -> constant yaw torque",
    "high_gain": "attitude loop too aggressive -> wobble",
    "low_gain": "attitude loop too soft -> sluggish, overshoot after braking",
    "wind": "steady 8 m/s wind from the west + gusts",
    "vibration": "unbalanced prop -> strong 85 Hz gyro vibration",
    "gps_glitch": "GPS position jumps 12 m for 3 s mid-flight",
    "brownout": "power lost in the air -> log ends while flying",
    "motor_failure": "M2 stops in flight",
    "weak_battery": "high internal resistance -> big voltage sag",
    "overweight": "40% heavier than design -> high throttle / saturation",
    "hard_landing": "final descent at 3.5 m/s into the ground",
    "esc_desync": "M4 output drops out randomly for 60-120 ms",
}

G = 9.80665
# body positions (x fwd, y right) of M1..M4 and spin sign (yaw torque sign)
POS = np.array([[1, 1], [1, -1], [-1, -1], [-1, 1]], float) * 0.177
SPIN = np.array([1, -1, 1, -1], float)


def euler_to_quat(r, p, y):
    cr, sr, cp, sp, cy, sy = (math.cos(r / 2), math.sin(r / 2), math.cos(p / 2), math.sin(p / 2),
                              math.cos(y / 2), math.sin(y / 2))
    return (cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy)


def rot(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cp * cy, sr * sp * cy - cr * sy, cr * sp * cy + sr * sy],
                     [cp * sy, sr * sp * sy + cr * cy, cr * sp * sy - sr * cy],
                     [-sp, sr * cp, cr * cp]])


def mission():
    """(time, N, E, alt, yaw_deg) setpoints."""
    return [(0, 0, 0, -1, 90), (4, 0, 0, 20, 90), (14, 0, 0, 20, 90),
            (16, 120, 0, 20, 90), (32, 120, 80, 20, 90), (46, 0, 80, 20, 180),
            (62, 0, 0, 20, 90), (76, 0, 0, 20, 90), (78, 0, 0, -1, 90), (200, 0, 0, -1, 90)]


def simulate(path, fault="none", seed=1, duration=None):
    rng = np.random.default_rng(seed)
    dt = 0.005
    mass = 2.0 * (1.4 if fault == "overweight" else 1.0)
    I = np.array([0.03, 0.03, 0.05])
    tmax = 2.0 * G / 4 / 0.55            # hover at 55% for the design mass
    cd = 0.5 * 1.225 * 0.06
    kq = 0.016
    eff = np.ones(4)
    if fault == "weak_motor":
        eff[2] = 0.82
    if fault == "motor_tilt":
        extra_yaw = 0.06
    else:
        extra_yaw = 0.0
    dist_torque = np.array([0.18, 0.14, 0.0]) if fault == "cg_offset" else np.zeros(3)
    kr = {"high_gain": 4.5, "low_gain": 0.6}.get(fault, 1.0)
    kang = 2.0 if fault == "low_gain" else 5.0
    wind = np.array([0.0, 8.0, 0.0]) if fault == "wind" else np.zeros(3)   # blows towards East = FROM West
    r_int = 0.10 if fault == "weak_battery" else 0.018
    b0 = (8.0, 8.0, -1.3)

    pos = np.zeros(3); vel = np.zeros(3); att = np.array([0.0, 0.0, math.radians(90)]); rate = np.zeros(3)
    thrust = np.zeros(4)
    integ = np.zeros(3); vinteg = np.zeros(3)
    delay = [np.zeros(3)] * 3
    sp = mission()
    t_end = duration or (90.0 if fault != "brownout" else 41.0)
    armed_t = 1.0
    out = [b"AC\x00\x03RLog202601011200" + b"\0" * 8]
    tick0 = 5_000_000
    soc_v = 25.0
    gps_off = np.zeros(3)
    desync_until = -1
    gust = np.zeros(3)
    motor_dead = None
    yaw_cmd = math.radians(90)
    k = 0
    t = 0.0
    while t < t_end:
        # ---- setpoint (linear interpolation between mission points)
        for a, b in zip(sp[:-1], sp[1:]):
            if a[0] <= t < b[0]:
                f = (t - a[0]) / (b[0] - a[0])
                tgt = np.array(a[1:4]) + f * (np.array(b[1:4]) - np.array(a[1:4]))
                yaw_sp = math.radians(a[4] + f * ((b[4] - a[4] + 180) % 360 - 180))
                if b[1:3] != a[1:3]:   # hold heading during legs
                    yaw_sp = math.radians(a[4])
                break
        tgt_ned = np.array([tgt[0], tgt[1], -tgt[2]])
        if fault == "hard_landing" and t > 76:
            tgt_ned[2] = 5.0
        if fault == "gps_glitch":
            gps_off = np.array([12.0, -6.0, 0.0]) if 30 < t < 33 else np.zeros(3)
        if fault == "wind":
            gust = 0.97 * gust + 0.03 * rng.normal(0, 3.0, 3) * np.array([1, 1, 0.3])
        dyaw = (yaw_sp - yaw_cmd + math.pi) % (2 * math.pi) - math.pi
        yaw_cmd += float(np.clip(dyaw, -math.radians(30) * dt, math.radians(30) * dt))
        yaw_sp = yaw_cmd
        meas_pos = pos + gps_off
        # ---- position / velocity loop
        armed = t > armed_t
        flying_cmd = tgt[2] > 0
        err = tgt_ned - meas_pos
        vsp = 0.8 * err
        h = np.hypot(vsp[0], vsp[1])
        if h > 10:
            vsp[:2] *= 10 / h
        vsp[2] = np.clip(vsp[2], -2.5, 1.5)
        if fault == "hard_landing" and t > 76:
            vsp[2] = 3.5
        ve = vsp - vel
        vinteg += ve * dt * (1 if pos[2] < -0.5 else 0)
        vinteg = np.clip(vinteg, -3, 3)
        acc_sp = 1.8 * ve + 0.6 * vinteg
        yaw = att[2]
        axb = math.cos(yaw) * acc_sp[0] + math.sin(yaw) * acc_sp[1]
        ayb = -math.sin(yaw) * acc_sp[0] + math.cos(yaw) * acc_sp[1]
        pitch_sp = float(np.clip(-math.atan2(axb, G), -0.6, 0.6))
        roll_sp = float(np.clip(math.atan2(ayb, G), -0.6, 0.6))
        coll = mass * (G - acc_sp[2]) / max(0.5, math.cos(att[0]) * math.cos(att[1]))
        if not flying_cmd and pos[2] > -0.3:
            coll = 0.0
        # ---- attitude loop (P on angle -> rate, PI on rate, with 15 ms delay)
        att_err = np.array([roll_sp - att[0], pitch_sp - att[1], (yaw_sp - att[2] + math.pi) % (2 * math.pi) - math.pi])
        rate_sp = np.array([kang, kang, 2.5]) * att_err
        rate_meas = rate.copy()
        re = rate_sp - rate_meas
        integ += re * dt * (1 if pos[2] < -0.5 else 0)
        integ = np.clip(integ, -2, 2)
        u_acc = kr * np.array([14.0, 14.0, 6.0]) * re + np.array([25.0, 25.0, 8.0]) * integ  # desired ang. accel
        delay.append(u_acc); u_acc = delay.pop(0)
        # mixing: angular accel -> per-motor thrust deltas
        dT_roll = I[0] * u_acc[0] / (4 * 0.177)
        dT_pitch = I[1] * u_acc[1] / (4 * 0.177)
        dT_yaw = I[2] * u_acc[2] / (4 * kq)
        base = coll / 4
        cmdT = base - dT_roll * np.sign(POS[:, 1]) + dT_pitch * np.sign(POS[:, 0]) + dT_yaw * SPIN
        pwm = 1000 + 962 * cmdT / tmax
        if armed:
            pwm = np.clip(pwm, 1250, 1962)
        else:
            pwm = np.full(4, 1000.0)
        if not flying_cmd and pos[2] > -0.3 and armed:
            pwm[:] = 1250 if t < 85 else 1000
        # ---- actuators
        target = tmax * np.clip((pwm - 1000) / 962, 0, 1) * eff
        if pwm.mean() <= 1260:
            target *= 0.1
        if fault == "esc_desync" and t > 20 and rng.random() < 0.0004:
            desync_until = t + rng.uniform(0.03, 0.06)
        if t < desync_until:
            target[3] = 0.0
        if fault == "motor_failure" and t > 40:
            target[1] = 0.0
            motor_dead = 1
        thrust += (target - thrust) * dt / 0.03
        # ---- rigid body
        torque = np.array([-(thrust * POS[:, 1]).sum(), (thrust * POS[:, 0]).sum(),
                           (thrust * SPIN).sum() * kq + extra_yaw]) + dist_torque
        if pos[2] > -0.05 and thrust.sum() < mass * G:   # on the ground
            torque[:] = 0; rate[:] = 0
        rate += (torque - np.cross(rate, I * rate)) / I * dt
        ph, th = att[0], att[1]
        pp, qq, rr = rate
        att += dt * np.array([pp + (qq * math.sin(ph) + rr * math.cos(ph)) * math.tan(th),
                              qq * math.cos(ph) - rr * math.sin(ph),
                              (qq * math.sin(ph) + rr * math.cos(ph)) / math.cos(th)])
        R = rot(*att)
        vair = vel - wind - gust
        acc = R @ np.array([0, 0, -thrust.sum()]) / mass + np.array([0, 0, G]) - cd * np.linalg.norm(vair) * vair / mass
        vel += acc * dt
        pos += vel * dt
        impact = 0.0
        if pos[2] > 0:
            impact = vel[2]
            pos[2] = 0; vel[:] = 0
            att[:2] *= 0.9
            acc_log = np.array([0, 0, -impact / dt * 0.5])
        else:
            acc_log = acc - np.array([0, 0, G]) + R @ np.array([0, 0, 0]) * 0
            acc_log = acc
        # ---- logging
        tick = tick0 + int(round(t / 1e-4))
        gyro = rate.copy()
        noise = 0.02 if armed and thrust.sum() > 0.5 else 0.002
        gyro += rng.normal(0, noise, 3)
        if fault == "vibration" and armed:
            gyro += 0.45 * np.array([1, 0.8, 0.3]) * math.sin(2 * math.pi * 85 * t + rng.normal(0, 0.2))
        q = euler_to_quat(*att)
        out.append(_pkt(0x1F, tick, struct.pack("<7d", *q, *gyro)))
        lp = meas_pos
        out.append(_pkt(0x20, tick, struct.pack("<9d", *lp, *vel, *acc_log)))
        for i, (ax, b) in enumerate(zip(("rol", "pit", "yaw"), b0)):
            u = float(u_acc[i]) / 10.0
            vec = [rate[i], rate[i], rate[i], rate_sp[i], att_err[i], b * u, u, -b * u, u, 0.0]
            out.append(_pkt(0xFA, tick, (ax.encode() + b"\0" * 10)[:10] + struct.pack("<10d", *vec), b"\x11\x11"))
        if k % 10 == 0:
            dj = list(pwm if motor_dead is None else pwm) + [60000.0] * 10
            out.append(_pkt(0xFA, tick, b"dianji\0\xff\xff\xff" + struct.pack("<14d", *dj), b"\xff\xff"))
        if k % 20 == 0:
            lat = 19.44 + meas_pos[0] / 111320
            lon = 74.19 + meas_pos[1] / (111320 * math.cos(math.radians(19.44)))
            gps = struct.pack("<10d", 19.44, 74.19, lat, lon, meas_pos[0], meas_pos[1], -800 + meas_pos[2], *vel)
            out.append(_pkt(0x1C, tick, bytes([135, 0, 14, 3, 0, 0, 0, 0]) + gps))
        if k % 40 == 0:
            cur = 1.5 + 0.27 * (thrust.sum() ** 1.5)
            soc_v -= cur * 0.2 / 3600 * 0.6
            v = soc_v - r_int * cur
            out.append(_pkt(0x01, tick, b"\x27\x00\x61\x01" + struct.pack("<10f", v, *([0.0] * 9)) + b"\0" * 8))
        t += dt
        k += 1
    with open(path, "wb") as fo:
        fo.write(b"".join(out))
    return path


def _pkt(pid, tick, payload, pre=b"\x00\x00"):
    body = pre + struct.pack("<I", tick) + payload
    return b"AC" + bytes([pid, len(body) + 2]) + body


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("out", nargs="?")
    ap.add_argument("--fault", default="none", choices=list(FAULTS))
    ap.add_argument("--list", action="store_true")
    a = ap.parse_args()
    if a.list or not a.out:
        for k, v in FAULTS.items():
            print(f"{k:14s} {v}")
    else:
        print(simulate(a.out, a.fault))
