"""Round-trip tests: write synthetic logs in the real formats, read them back."""
import math
import os
import struct
import sys
import tempfile

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from dronelog_qc.cli import load_config  # noqa: E402
from dronelog_qc.analysis.core import analyze  # noqa: E402
from dronelog_qc.loaders import load_any  # noqa: E402


def _pkt(pid, tick, payload, pre=b"\x00\x00"):
    body = pre + struct.pack("<I", tick) + payload
    return b"AC" + bytes([pid, len(body) + 2]) + body


def make_rlog(path, seconds=30.0):
    out = [b"AC\x00\x03RLog202601010000" + b"\0" * 8]
    tick0 = 6_000_000
    dt = 0.005
    alt = 0.0
    for k in range(int(seconds / dt)):
        t = k * dt
        tick = tick0 + int(round(t / 1e-4))
        flying = 5 < t < 25
        vz = 2.0 if 5 < t < 10 else (-2.0 if 20 < t < 25 else 0.0)
        alt += vz * dt
        vn = 3.0 if 12 < t < 18 else 0.0
        yaw = math.radians(90)
        roll = math.radians(2 * math.sin(2 * math.pi * 0.5 * t)) if flying else 0.0
        cr, sr, cy, sy = math.cos(roll / 2), math.sin(roll / 2), math.cos(yaw / 2), math.sin(yaw / 2)
        q = (cr * cy, sr * cy, -sr * sy, cr * sy)
        rate = math.radians(2 * 2 * math.pi * 0.5 * math.cos(2 * math.pi * 0.5 * t)) if flying else 0.0
        out.append(_pkt(0x1F, tick, struct.pack("<7d", *q, rate, 0.0, 0.0)))
        out.append(_pkt(0x20, tick, struct.pack("<9d", vn * max(0, t - 12) if t < 18 else 18.0, 0, -alt,
                                                vn, 0, -vz, 0, 0, 0)))
        if k % 10 == 0:
            pwm = 1550.0 if flying else 1250.0
            name = b"dianji\0\xff\xff\xff"
            out.append(_pkt(0xFA, tick, name + struct.pack("<14d", pwm, pwm, pwm, pwm, *([60000.0] * 10)), b"\xff\xff"))
        if k % 40 == 0:
            out.append(_pkt(0x01, tick, b"\x27\x00\x61\x01" + struct.pack("<10f", 24.5 - 0.02 * t, *([0.0] * 9)) + b"\0" * 8))
    with open(path, "wb") as f:
        f.write(b"".join(out))


def test_rlog_roundtrip():
    with tempfile.TemporaryDirectory() as d:
        p = os.path.join(d, "x.rlog")
        make_rlog(p)
        fd = load_any(p)
        assert fd.meta["tick_seconds"] == 1e-4
        assert fd.meta["file_header"].startswith("RLog2026")
        assert abs(fd.time_range()[1] - 30.0) < 0.1
        assert fd.motor_names() == ["motor_1", "motor_2", "motor_3", "motor_4"]
        _, yaw = fd.get("yaw")
        assert abs(np.median(yaw) - 90) < 1
        _, alt = fd.get("alt_rel")
        assert 9 < alt.max() < 11
        res = analyze(fd, load_config())
        assert 4 < res.summary["takeoff_t"] < 7
        assert not res.upset.get("t_impact")


def test_tlog_reads():
    from pymavlink.dialects.v20 import ardupilotmega as mav
    with tempfile.TemporaryDirectory() as d:
        p = os.path.join(d, "x.tlog")
        m = mav.MAVLink(None, srcSystem=1, srcComponent=1)
        with open(p, "wb") as f:
            for k in range(400):
                tb = 1000 + k * 50
                wall = int((1.7e9 + k * 0.05) * 1e6)
                msgs = [mav.MAVLink_heartbeat_message(2, 3, 128 | 1, 4, 4, 3),
                        mav.MAVLink_attitude_message(tb, 0.01, -0.02, 1.0, 0, 0, 0),
                        mav.MAVLink_global_position_int_message(tb, 194402600, 741907300, 800000, 5000 if k > 50 else 0,
                                                               100, 0, 0, 9000),
                        mav.MAVLink_servo_output_raw_message(tb * 1000, 0, *([1500 + (k % 7)] * 4), *([0] * 4))]
                for msg in msgs:
                    f.write(struct.pack(">Q", wall) + msg.pack(m))
        fd = load_any(p)
        assert fd.has("roll") and fd.has("motor_1") and fd.has("pos_n")
        assert abs(np.median(fd.get("roll")[1]) - math.degrees(0.01)) < 0.01
        analyze(fd, load_config())


if __name__ == "__main__":
    test_rlog_roundtrip(); test_tlog_reads(); print("ok")
