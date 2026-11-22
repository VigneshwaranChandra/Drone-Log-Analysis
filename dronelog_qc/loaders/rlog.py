"""RFly binary log (.rlog) loader - reverse engineered from a real sample.

Format (all little-endian), verified on 55b8d82c-202511081618.rlog:

  packet := 'A' 'C' | id:u8 | len:u8 | body[len-2]
            (len counts id+len+body, i.e. next packet starts at offset+2+len)
  body   := pre:u16 | tick:u32 | payload
            tick unit = 100 us  (verified: velocity == d(position)/dt and
            gyro == quaternion derivative only with this scale)

  id 0x00  file header  "RLog" + yyyymmddHHMM
  id 0x1f  attitude, 7 x f64: quaternion w,x,y,z (world NED -> body FRD),
           gyro p,q,r [rad/s]                                ~200 Hz
  id 0x20  kinematics, 9 x f64: pos N,E,D [m], vel N,E,D [m/s],
           acc N,E,D [m/s^2] (earth frame, gravity removed)  ~200 Hz
  id 0x1c  len 96: GPS. bytes[0]=sub-id 135, [2]=satellites, [3]=fix type,
           then 10 x f64: home lat, home lon, lat, lon, N, E, D(-MSL alt),
           vN, vE, vD                                       ~10 Hz
  id 0x1c  len 40: sub-id 132, meaning unknown (kept raw)
  id 0x01  power: 4 bytes + 10 x f32 (+2 x u32); f32[0] = battery voltage
  id 0xfa  named debug vector ("DVecN name" in the RflySim viewer):
           name: 10 bytes (NUL terminated, rest is garbage), then N x f64
             dianji   (motor = 电机) 14 values, [0..3] = motor PWM, 60000 = unused
             lingpian (零偏 = zero bias) 3 values
             rol/pit/yaw  10 values: ADRC internals per axis.
                 [6] = controller output u (explains motor differences, R^2 0.98)
                 [5] = b0 * u  -> [5]/[6] gives the b0 that was set
                       (yaw: -1.2 exactly, matches the GCS value)
                 other elements: not decoded yet (kept as adrc_<axis>_<k>)

Unknown packet types are counted and reported, never silently dropped.
"""
from __future__ import annotations

import math
import struct
from collections import Counter, defaultdict

import numpy as np

from ..data import Event, FlightData

DEFAULT_TICK_S = 1e-4


def _packets(buf: bytes):
    i, n, skipped = 0, len(buf), 0
    out = []
    while i < n - 4:
        if buf[i] == 0x41 and buf[i + 1] == 0x43:
            ln = buf[i + 3]
            end = i + 2 + ln
            if ln >= 2 and end <= n and (end == n or buf[end:end + 2] == b"AC" or end + 1 >= n):
                out.append((buf[i + 2], ln, buf[i + 4:end]))
                i = end
                continue
        skipped += 1
        i += 1
    return out, skipped


def _detect_tick(kin_t_ticks: np.ndarray, kin: np.ndarray) -> float:
    """Pick the tick length so that velocity == d(position)/dt."""
    best, best_err = DEFAULT_TICK_S, 1e9
    for tick in (1e-3, 1e-4, 1e-5, 1e-6):
        t = kin_t_ticks * tick
        if np.ptp(t) <= 0:
            continue
        ratios = []
        for a in range(3):
            dp = np.gradient(kin[:, a], t)
            v = kin[:, 3 + a]
            m = np.abs(v) > 1.0
            if m.sum() > 50:
                ratios.append(np.median(v[m] / np.where(dp[m] == 0, np.nan, dp[m])))
        ratios = [r for r in ratios if np.isfinite(r)]
        if not ratios:
            continue
        err = abs(math.log(abs(np.median(ratios))))
        if err < best_err:
            best, best_err = tick, err
    return best


def load_rlog(path: str) -> FlightData:
    with open(path, "rb") as f:
        buf = f.read()
    pk, skipped = _packets(buf)
    fd = FlightData(source=path)
    raw = defaultdict(list)
    counts = Counter()
    header = ""
    vec_names = Counter()
    for pid, ln, body in pk:
        counts[(pid, ln)] += 1
        if pid == 0x00:
            header = body.split(b"\0")[0].decode("ascii", "replace")
            continue
        if len(body) < 6:
            continue
        tick = struct.unpack_from("<I", body, 2)[0]
        p = body[6:]
        try:
            if pid == 0x1F and len(p) == 56:
                raw["att"].append((tick,) + struct.unpack("<7d", p))
            elif pid == 0x20 and len(p) == 72:
                raw["kin"].append((tick,) + struct.unpack("<9d", p))
            elif pid == 0x1C and len(p) == 88:
                raw["gps"].append((tick, p[2], p[3]) + struct.unpack_from("<10d", p, 8))
            elif pid == 0x1C and len(p) == 32:
                raw["x1c40"].append((tick,) + tuple(p[:8]) + struct.unpack_from("<d", p, 24))
            elif pid == 0x01 and len(p) >= 44:
                raw["pwr"].append((tick,) + struct.unpack_from("<10f", p, 4))
            elif pid == 0xFA and len(p) > 10:
                name = p[:10].split(b"\0")[0].decode("ascii", "replace")
                nv = (len(p) - 10) // 8
                vec_names[f"DVec{nv} {name}"] += 1
                raw["v_" + name].append((tick,) + struct.unpack_from(f"<{nv}d", p, 10))
        except struct.error:
            counts[("struct_error", pid)] += 1

    arr = {k: np.array(v, dtype=float) for k, v in raw.items() if v}
    if "kin" in arr:
        tick_s = _detect_tick(arr["kin"][:, 0], arr["kin"][:, 1:])
    else:
        tick_s = DEFAULT_TICK_S

    def T(a):
        return a[:, 0] * tick_s

    if "att" in arr:
        a = arr["att"]
        t = T(a)
        w, x, y, z = a[:, 1], a[:, 2], a[:, 3], a[:, 4]
        fd.add("roll", t, np.degrees(np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))))
        fd.add("pitch", t, np.degrees(np.arcsin(np.clip(2 * (w * y - z * x), -1, 1))))
        fd.add("yaw", t, np.degrees(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))) % 360)
        for i, n in enumerate(("qw", "qx", "qy", "qz")):
            fd.add(n, t, a[:, 1 + i])
        for i, n in enumerate(("p", "q", "r")):
            fd.add(n, t, np.degrees(a[:, 5 + i]))
    if "kin" in arr:
        a = arr["kin"]
        t = T(a)
        fd.add("pos_n", t, a[:, 1]); fd.add("pos_e", t, a[:, 2]); fd.add("pos_u", t, -a[:, 3])
        fd.add("vel_n", t, a[:, 4]); fd.add("vel_e", t, a[:, 5]); fd.add("vel_u", t, -a[:, 6])
        fd.add("accw_n", t, a[:, 7]); fd.add("accw_e", t, a[:, 8]); fd.add("accw_u", t, -a[:, 9])
        fd.add("alt_rel", t, -(a[:, 3] - a[0, 3]))
    if "gps" in arr:
        a = arr["gps"]
        t = T(a)
        fd.add("gps_sats", t, a[:, 1]); fd.add("gps_fix", t, a[:, 2])
        fd.add("lat", t, a[:, 5]); fd.add("lon", t, a[:, 6])
        fd.add("alt_msl", t, -a[:, 9])
        fd.meta["home"] = (float(a[0, 3]), float(a[0, 4]))
    if "pwr" in arr:
        a = arr["pwr"]
        fd.add("volt", T(a), a[:, 1])
    if "v_dianji" in arr:
        a = arr["v_dianji"]
        t = T(a)
        k = 1
        for i in range(1, a.shape[1]):
            col = a[:, i]
            if np.all(col == 60000) or np.all(col == 0):
                continue
            fd.add(f"motor_{k}", t, col)
            k += 1
        fd.meta["motor_units"] = "PWM_us"
    for ax in ("rol", "pit", "yaw"):
        key = "v_" + ax
        if key in arr:
            a = arr[key]
            t = T(a)
            for i in range(1, a.shape[1]):
                fd.add(f"adrc_{ax}_{i - 1}", t, a[:, i])
    if "v_lingpian" in arr:
        fd.meta["lingpian_zero_bias_last"] = [round(float(v), 4) for v in arr["v_lingpian"][-1, 1:]]
    if "x1c40" in arr:
        a = arr["x1c40"]
        fd.add("x1c40_b2", T(a), a[:, 3]); fd.add("x1c40_b3", T(a), a[:, 4])

    # Armed state from motors: motor > 1100 us == spinning (1000 = stopped)
    if fd.has("motor_1"):
        t, _ = fd.get("motor_1")
        mm = np.vstack([fd.get(n)[1] for n in fd.motor_names()])
        spinning = mm.min(axis=0) > 1100     # all motors spinning
        for i in np.flatnonzero(np.diff(spinning.astype(int)) != 0):
            fd.events.append(Event(float(t[i + 1]), "arm" if spinning[i + 1] else "disarm",
                                   "Motors started (armed)" if spinning[i + 1] else "Motors stopped"))

    mh = __import__("re").search(rb"RLog\d{8,14}", buf[:64])
    if mh:
        header = mh.group().decode()
    t0 = min(s[0][0] for s in fd.series.values()) if fd.series else 0.0
    fd.meta.update({
        "format": "RFly binary log (.rlog)",
        "file_header": header,
        "tick_seconds": tick_s,
        "boot_time_at_log_start_s": round(t0, 2),
        "frames": "attitude: quaternion NED->FRD; position/velocity: NED (stored here as N,E,Up)",
        "packet_counts": {f"0x{k[0]:02x}/len{k[1]}" if isinstance(k[0], int) else str(k): v
                          for k, v in counts.items()},
        "debug_vectors": dict(vec_names),
        "skipped_bytes": skipped,
        "autopilot_name": "RFly (custom, ADRC attitude control)",
    })
    fd.shift_time(t0)
    return fd
