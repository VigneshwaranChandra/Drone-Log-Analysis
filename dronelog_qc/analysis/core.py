"""Flight analysis engine.

Every finding carries: severity (OK / INFO / WARN / BAD), a plain-language
message, the evidence numbers, and a confidence (high / medium / low).
Rule: anything that is an interpretation, not a measurement, must say so.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional

import numpy as np

from ..data import FlightData

G = 9.80665


@dataclass
class Finding:
    area: str
    severity: str          # OK / INFO / WARN / BAD
    title: str
    detail: str
    confidence: str = "high"
    t: Optional[float] = None
    issue: Optional[str] = None      # key into knowledge.ISSUES


@dataclass
class Result:
    fd: FlightData
    cfg: dict
    summary: Dict[str, object] = field(default_factory=dict)
    segments: List[dict] = field(default_factory=list)
    findings: List[Finding] = field(default_factory=list)
    motors: Dict[str, object] = field(default_factory=dict)
    control: Dict[str, object] = field(default_factory=dict)
    vibration: Dict[str, object] = field(default_factory=dict)
    upset: Dict[str, object] = field(default_factory=dict)
    wind: Dict[str, object] = field(default_factory=dict)
    power: Dict[str, object] = field(default_factory=dict)
    tuning: List[dict] = field(default_factory=list)
    grid_t: np.ndarray = None
    grid: Dict[str, np.ndarray] = None

    def add(self, *a, **k):
        self.findings.append(Finding(*a, **k))


# ---------------------------------------------------------------- helpers
def _smooth(x, n):
    n = max(1, int(n))
    if n == 1:
        return x
    k = np.ones(n) / n
    xp = np.pad(x, (n // 2, n - 1 - n // 2), mode="edge")
    return np.convolve(xp, k, mode="valid")


def _runs(mask):
    """Return list of (start_idx, end_idx_exclusive) of True runs."""
    m = np.concatenate([[False], np.asarray(mask, bool), [False]])
    d = np.diff(m.astype(int))
    return list(zip(np.flatnonzero(d == 1), np.flatnonzero(d == -1)))


def _highpass_rms(t, x, fc):
    """RMS of x above cutoff fc (Hz). Butterworth if scipy is there, else moving-average subtraction."""
    if len(x) < 10:
        return float("nan")
    fs = 1.0 / np.median(np.diff(t))
    try:
        from scipy.signal import butter, sosfiltfilt
        if fc < fs / 2.2 and len(x) > 30:
            sos = butter(4, fc, "highpass", fs=fs, output="sos")
            return float(np.sqrt(np.mean(sosfiltfilt(sos, x - np.mean(x)) ** 2)))
    except Exception:
        pass
    n = max(3, int(fs / fc))
    return float(np.sqrt(np.mean((x - _smooth(x, n)) ** 2)))


def _psd_peak(t, x, fmin=0.5, fmax=None):
    if len(x) < 64:
        return float("nan"), float("nan")
    fs = 1.0 / np.median(np.diff(t))
    x = x - np.mean(x)
    nseg = min(len(x), int(2 ** math.floor(math.log2(min(len(x), fs * 4)))))
    try:
        from scipy.signal import welch
        f, pxx = welch(x, fs=fs, nperseg=nseg)
    except Exception:
        f = np.fft.rfftfreq(len(x), 1 / fs)
        pxx = np.abs(np.fft.rfft(x)) ** 2
    fmax = fmax or fs / 2
    m = (f >= fmin) & (f <= fmax)
    if not m.any():
        return float("nan"), float("nan")
    i = np.argmax(pxx[m])
    return float(f[m][i]), float(pxx[m][i])


def _quat_R(fd, t):
    qs = [np.interp(t, *fd.get(n)) for n in ("qw", "qx", "qy", "qz")]
    w, x, y, z = qs
    nrm = np.sqrt(w * w + x * x + y * y + z * z)
    w, x, y, z = w / nrm, x / nrm, y / nrm, z / nrm
    # body (FRD) -> world (NED)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


# ---------------------------------------------------------------- main
def analyze(fd: FlightData, cfg: dict) -> Result:
    res = Result(fd=fd, cfg=cfg)
    rate = 50.0
    t, g = fd.grid(rate)
    res.grid_t, res.grid = t, g
    _summary(res)
    _phases(res)
    _upset_and_impact(res)
    _motors(res)
    _vibration(res)
    _control(res)
    _attitude_consistency(res)
    _power(res)
    from . import diagnostics
    diagnostics.run(res)
    _tuning_advice(res)
    order = {"BAD": 0, "WARN": 1, "INFO": 2, "OK": 3}
    res.findings.sort(key=lambda f: (order.get(f.severity, 9), f.t if f.t is not None else 0))
    return res


# ---------------------------------------------------------------- summary
def _airborne_mask(res):
    t, g = res.grid_t, res.grid
    fd = res.fd
    motors = [g[n] for n in fd.motor_names() if n in g]
    air = np.zeros_like(t, bool)
    if "alt_rel" in g:
        air |= np.nan_to_num(g["alt_rel"]) > 1.0
    if motors:
        mm = np.nanmean(np.vstack(motors), axis=0)
        idle = res.cfg["airframe"].get("pwm_idle", 1250)
        air &= mm > idle + 30
    return air


def _summary(res):
    fd, t, g = res.fd, res.grid_t, res.grid
    s = res.summary
    s["log_duration_s"] = float(t[-1] - t[0]) if len(t) else 0.0
    air = _airborne_mask(res)
    res.grid["_air"] = air.astype(float)
    runs = _runs(air)
    runs = [r for r in runs if (r[1] - r[0]) > 25]   # >0.5 s
    if runs:
        s["takeoff_t"] = float(t[runs[0][0]])
        s["landing_t"] = float(t[runs[-1][1] - 1])
        s["air_time_s"] = float(sum(t[b - 1] - t[a] for a, b in runs))
    arms = [e.t for e in fd.events if e.kind == "arm"]
    s["arm_t"] = arms[0] if arms else None
    if "alt_rel" in g:
        s["max_alt_m"] = float(np.nanmax(g["alt_rel"]))
        s["min_alt_m"] = float(np.nanmin(g["alt_rel"]))
    if "vel_n" in g and "vel_e" in g:
        hs = np.hypot(g["vel_n"], g["vel_e"])
        g["_hspeed"] = hs
        s["max_hspeed_ms"] = float(np.nanmax(hs))
        s["distance_m"] = float(np.nansum(hs) / 50.0)
    if "vel_u" in g:
        s["max_climb_ms"] = float(np.nanmax(g["vel_u"]))
        s["max_descent_ms"] = float(-np.nanmin(g["vel_u"]))
    if fd.has("lat"):
        _, la = fd.get("lat"); _, lo = fd.get("lon")
        s["location"] = f"{np.median(la):.5f}, {np.median(lo):.5f}"
    if fd.has("gps_sats"):
        s["gps_sats_min"] = int(np.min(fd.get("gps_sats")[1]))
    s["boot_time_at_log_start_s"] = fd.meta.get("boot_time_at_log_start_s")


# ---------------------------------------------------------------- phases
def _phases(res):
    t, g = res.grid_t, res.grid
    if "_hspeed" not in g:
        return
    air = g["_air"] > 0.5
    hs = _smooth(np.nan_to_num(g["_hspeed"]), 25)
    vz = _smooth(np.nan_to_num(g.get("vel_u", np.zeros_like(t))), 25)
    along = np.gradient(hs, t)
    along = _smooth(along, 25)
    yawrate = np.zeros_like(t)
    if "r" in g:
        yawrate = _smooth(np.nan_to_num(g["r"]), 25)
    course = np.unwrap(np.arctan2(np.nan_to_num(g["vel_e"]), np.nan_to_num(g["vel_n"])))
    crate = np.degrees(_smooth(np.gradient(course, t), 25))
    crate[hs < 2] = 0
    tilt = np.zeros_like(t)
    if "roll" in g:
        tilt = np.degrees(np.arccos(np.clip(np.cos(np.radians(g["roll"])) * np.cos(np.radians(g["pitch"])), -1, 1)))
    lab = np.full(t.shape, "ground", dtype=object)
    lab[air] = "hover"
    lab[air & (hs > 2)] = "cruise"
    lab[air & (np.abs(crate) > 12) & (hs > 2)] = "turn"
    lab[air & (np.abs(yawrate) > 25) & (hs <= 2)] = "yaw-turn"
    lab[air & (along > 1.0) & (hs > 1)] = "accelerating"
    lab[air & (along < -1.0) & (hs > 1)] = "braking"
    lab[air & (vz > 0.8)] = "climb"
    lab[air & (vz < -0.8)] = "descent"
    lab[air & (tilt > res.cfg["thresholds"]["upset_angle_deg"])] = "UPSET"
    # merge short pieces (< 0.6 s) into previous label
    segs = []
    start = 0
    for i in range(1, len(lab) + 1):
        if i == len(lab) or lab[i] != lab[start]:
            segs.append([start, i, lab[start]])
            start = i
    merged = []
    for a, b, l in segs:
        if merged and (b - a) < 30 and l != "UPSET":
            merged[-1][1] = b
        elif merged and merged[-1][2] == l:
            merged[-1][1] = b
        else:
            merged.append([a, b, l])
    out = []
    for a, b, l in merged:
        sl = slice(a, b)
        d = {"t0": float(t[a]), "t1": float(t[b - 1]), "phase": l,
             "speed": float(np.nanmean(hs[sl])), "vz": float(np.nanmean(vz[sl])),
             "max_tilt": float(np.nanmax(tilt[sl])) if len(tilt[sl]) else 0.0}
        if "alt_rel" in g:
            d["alt"] = float(np.nanmean(g["alt_rel"][sl]))
        if hs[sl].mean() > 1:
            cdeg = (np.degrees(np.arctan2(np.nanmean(g["vel_e"][sl]), np.nanmean(g["vel_n"][sl])))) % 360
            d["course"] = float(cdeg)
        if "yaw" in g:
            d["heading"] = float(np.nanmedian(g["yaw"][sl]))
        out.append(d)
    res.segments = out
    g["_phase"] = lab


# ---------------------------------------------------------------- upset / crash
def _upset_and_impact(res):
    fd, t, g, th = res.fd, res.grid_t, res.grid, res.cfg["thresholds"]
    if "roll" not in g:
        return
    air = g["_air"] > 0.5
    tilt = np.degrees(np.arccos(np.clip(np.cos(np.radians(g["roll"])) * np.cos(np.radians(g["pitch"])), -1, 1)))
    g["_tilt"] = tilt
    idx = np.flatnonzero(air & (tilt > th["upset_angle_deg"]))
    up = res.upset
    if idx.size == 0:
        res.add("Flight", "OK", "No loss of attitude control",
                f"Max tilt while airborne {np.nanmax(np.where(air, tilt, 0)):.0f} deg.")
        return
    i_up = idx[0]
    t_up = float(t[i_up])
    # when did the excursion start? walk back while tilt keeps rising above 10 deg
    j = i_up
    while j > 0 and tilt[j - 1] > 8 and t[i_up] - t[j] < 5:
        j -= 1
    t_start = float(t[j])
    up.update({"t_upset": t_up, "t_start": t_start, "tilt_peak": float(np.nanmax(tilt[i_up:i_up + 150]))})
    # which way did it go? (log axes)
    r0, p0 = g["roll"][i_up], g["pitch"][i_up]
    up["roll_at_upset"], up["pitch_at_upset"] = float(r0), float(p0)
    # motor behaviour during the excursion (start .. upset)
    names = fd.motor_names()
    sl = slice(j, i_up + 1)
    if names:
        M = np.vstack([g[n][sl] for n in names])
        spread = (np.nanmax(M, axis=0) - np.nanmin(M, axis=0))
        mean = np.nanmean(M, axis=0)
        up["motor_spread_during_dive_pwm"] = float(np.nanmax(spread))
        pmax = res.cfg["airframe"]["pwm_max"]
        up["any_motor_at_max_during_dive"] = bool(np.nanmax(M) >= pmax - 5)
        # first saturation after start
        Mall = np.vstack([g[n] for n in names])
        sat = np.flatnonzero((np.nanmax(Mall, axis=0) >= pmax - 5) & (t >= t_start))
        up["first_saturation_t"] = float(t[sat[0]]) if sat.size else None
        rng = res.cfg["airframe"]["pwm_max"] - res.cfg["airframe"]["pwm_min"]
        up["motor_spread_pct"] = float(100 * np.nanmax(spread) / rng)
    # rate of the dive
    rates = []
    for a in ("p", "q"):
        if a in g:
            rates.append(np.nanmax(np.abs(g[a][sl])))
    up["peak_rate_dps"] = float(max(rates)) if rates else None
    # impact: biggest earth-frame acceleration after the upset
    t_imp = None
    if fd.has("accw_n"):
        tk, an = fd.get("accw_n"); _, ae = fd.get("accw_e"); _, au = fd.get("accw_u")
        mag = np.sqrt(an ** 2 + ae ** 2 + au ** 2)
        m = tk >= t_start
        if m.any():
            k = np.argmax(np.where(m, mag, 0))
            up["peak_accel_ms2"] = float(mag[k])
            if mag[k] > th["crash_accel_ms2"]:
                first = np.flatnonzero(m & (mag > th["crash_accel_ms2"]))
                t_imp = float(tk[first[0]])
                up["t_impact"] = t_imp
    # motors after impact
    if t_imp is not None and names:
        after = t > t_imp + 1.0
        if after.any():
            Mafter = np.vstack([g[n][after] for n in names])
            idle = res.cfg["airframe"]["pwm_idle"]
            spinning = np.nanmin(Mafter, axis=0) > res.cfg["airframe"]["pwm_min"] + 50
            up["motors_running_after_impact_s"] = float(spinning.sum() / 50.0)
            up["motor_mean_after_impact"] = [float(np.nanmean(x)) for x in Mafter]
    # Narrative
    spread_pct = up.get("motor_spread_pct", 0)
    detail = (f"Tilt went from <8 deg at t={t_start:.2f}s to {th['upset_angle_deg']:.0f} deg at t={t_up:.2f}s "
              f"(peak {up['tilt_peak']:.0f} deg; log roll {r0:.0f}, pitch {p0:.0f}). "
              f"Peak body rate {up.get('peak_rate_dps') or 0:.0f} deg/s. ")
    if names:
        detail += (f"During the dive the largest motor-to-motor difference was {up['motor_spread_during_dive_pwm']:.0f} us "
                   f"({spread_pct:.0f}% of range); motors at max: {'yes' if up['any_motor_at_max_during_dive'] else 'no'}.")
    res.add("Crash", "BAD", "Loss of attitude control (upset)", detail, "high", t_up)
    if names and spread_pct < 15 and not up["any_motor_at_max_during_dive"]:
        res.add("Crash", "BAD", "Controller did NOT fight the dive",
                "While the model tipped over, all motors stayed close together and none hit the limit. "
                "A failing motor/prop would make the controller push one side hard (big motor split / saturation) - that did not happen. "
                "So the likely causes are: (1) the attitude was COMMANDED (outer loop / mode / RC input asked for it), "
                "(2) the attitude estimate was wrong, or (3) a control-software fault. A prop/motor failure is LESS likely. "
                "This log has no attitude-setpoint channel, so (1) cannot be confirmed - please log the attitude setpoint and RC input.",
                "medium", t_start, issue="upset_uncommanded")
    if up.get("first_saturation_t"):
        res.add("Crash", "WARN", "Motors saturated after the upset",
                f"First motor at max at t={up['first_saturation_t']:.2f}s - the controller ran out of authority while recovering/falling.",
                "high", up["first_saturation_t"])
    if t_imp is not None:
        res.add("Crash", "BAD", "Ground impact detected",
                f"Acceleration {up['peak_accel_ms2']:.0f} m/s^2 ({up['peak_accel_ms2'] / G:.1f} g) at t={t_imp:.2f}s.", "high", t_imp)
    if up.get("motors_running_after_impact_s", 0) > 2:
        res.add("Safety", "BAD", "Motors kept running after the crash",
                f"Motors were still powered for {up['motors_running_after_impact_s']:.0f} s after impact "
                f"(mean PWM {', '.join(f'{v:.0f}' for v in up['motor_mean_after_impact'])}). "
                "The autopilot has no crash-detect auto-disarm (or it did not trigger). Risk of fire / injury - ask the vendor for crash disarm.",
                "high", t_imp)


# ---------------------------------------------------------------- motors
LAYOUT_SIGNS = {  # (roll torque sign: left side up positive, pitch torque sign: nose up positive)
    "front-right": (-1, +1), "front-left": (+1, +1), "rear-left": (+1, -1), "rear-right": (-1, -1),
}


def _motors(res):
    fd, t, g, cfg = res.fd, res.grid_t, res.grid, res.cfg
    names = fd.motor_names()
    if not names:
        res.add("Motors", "INFO", "No motor data in log", "Motor analysis skipped.")
        return
    af, th = cfg["airframe"], cfg["thresholds"]
    pmin, pmax = af["pwm_min"], af["pwm_max"]
    M = np.vstack([g[n] for n in names])
    air = g["_air"] > 0.5
    normal = air.copy()
    if res.upset.get("t_start") is not None:
        normal &= t < res.upset["t_start"] - 0.5      # keep the pre-upset disturbance out
    mo = res.motors
    mo["names"] = names
    mo["normal_window"] = (float(t[normal][0]), float(t[normal][-1])) if normal.any() else None
    if not normal.any():
        return
    Mn = M[:, normal]
    mean = np.nanmean(Mn, axis=1)
    mo["mean_pwm"] = mean.tolist()
    thr = 100 * (np.nanmean(Mn) - pmin) / (pmax - pmin)
    mo["avg_throttle_pct"] = float(thr)
    dev = 100 * (mean - mean.mean()) / (pmax - pmin)
    mo["deviation_pct"] = dev.tolist()
    sat_hi = 100 * np.mean(np.nanmax(Mn, axis=0) >= pmax - 5)
    sat_lo = 100 * np.mean(np.nanmin(Mn, axis=0) <= af["pwm_idle"] + 5)
    mo["sat_high_pct"], mo["sat_low_pct"] = float(sat_hi), float(sat_lo)
    mo["noise_pwm"] = [float(_highpass_rms(t[normal], x, 2.0)) for x in Mn]

    if thr <= th["hover_throttle_high_pct"]:
        res.add("Motors", "OK", "Throttle reserve", f"Average motor output {thr:.0f}% of range in normal flight.")

    # layout-based torque split
    lay = af.get("motor_layout", {})
    signs = [LAYOUT_SIGNS.get(lay.get(n, ""), None) for n in names[:4]]
    if len(names) == 4 and all(signs):
        rs = np.array([s[0] for s in signs]); ps = np.array([s[1] for s in signs])
        mo["roll_trim_pwm"] = float((rs * mean).sum() / 2)
        mo["pitch_trim_pwm"] = float((ps * mean).sum() / 2)
        # physics check: does (left-right) move roll and (front-rear) move pitch?
        mo["layout_check"] = _layout_check(res, names, rs, ps)
        lc = mo["layout_check"]
        if lc.get("verdict") == "mismatch":
            res.add("Motors", "WARN", "Motor layout in config does not match the physics in the log",
                    lc["text"], lc.get("confidence", "medium"), issue="layout_mismatch")
        elif lc.get("verdict") == "ok":
            res.add("Motors", "OK", "Motor layout confirmed by physics", lc["text"])


def _layout_check(res, names, rs, ps):
    fd = res.fd
    if not (fd.has("p") and fd.has("q")):
        return {"verdict": "unknown", "text": "no gyro"}
    tm, _ = fd.get(names[0])
    t0, t1 = res.motors["normal_window"]
    m = (tm >= t0) & (tm <= t1)
    up = res.upset
    if up.get("t_start") is not None and not up.get("any_motor_at_max_during_dive"):
        # the dive onset is the most informative part - unless a motor was saturated (then commands != thrust)
        m |= (tm >= t0) & (tm <= up["t_upset"])
    if m.sum() < 40:
        return {"verdict": "unknown", "text": "too little flight data"}
    Mm = np.vstack([fd.get(n)[1][m] for n in names[:4]])
    roll_cmd = rs @ Mm      # left - right
    pitch_cmd = ps @ Mm     # front - rear
    tq, _ = fd.get("p")
    acc = {}
    for a in ("p", "q"):
        tt, v = fd.get(a)
        fs = 1 / np.median(np.diff(tt))
        acc[a] = np.gradient(_smooth(_smooth(v, int(fs / 10) + 1), int(fs / 10) + 1), tt)
    def corr(x, a):
        best = 0.0
        for lag in np.arange(0, 0.2, 0.025):
            y = np.interp(tm[m] + lag, fd.get(a)[0], acc[a])
            c = np.corrcoef(x, y)[0, 1]
            if abs(c) > abs(best):
                best = c
        return float(best)
    c = {("roll", "p"): corr(roll_cmd, "p"), ("roll", "q"): corr(roll_cmd, "q"),
         ("pitch", "p"): corr(pitch_cmd, "p"), ("pitch", "q"): corr(pitch_cmd, "q")}
    straight = c[("roll", "p")] + c[("pitch", "q")]
    crossed = abs(c[("roll", "q")]) + abs(c[("pitch", "p")])
    txt = (f"corr(left-right -> roll accel) {c[('roll', 'p')]:+.2f}, corr(front-rear -> pitch accel) {c[('pitch', 'q')]:+.2f}, "
           f"cross terms {c[('roll', 'q')]:+.2f} / {c[('pitch', 'p')]:+.2f}.")
    if max(straight, crossed) < 0.5:
        return {"verdict": "unknown", "text": "Inconclusive (not enough motion). " + txt, "corr": c}
    if straight > crossed and c[("roll", "p")] > 0.2 and c[("pitch", "q")] > 0.2:
        return {"verdict": "ok", "text": txt, "corr": c}
    # find a layout that fits
    cand = _fit_layout(res, names, tm, m, acc)
    return {"verdict": "mismatch", "confidence": "medium", "corr": c, "suggested": cand,
            "text": ("The configured motor positions do not explain the measured rotations. " + txt +
                     (f" Best fit from the data: {cand}." if cand else "") +
                     " Either the motor order in the log (dianji[0..3]) differs from the M1..M4 labels, or the layout in config is wrong. "
                     "Bench test (props OFF): arm, tilt the model nose-down by hand and note which two dianji values rise - those are the FRONT motors.")}


def _fit_layout(res, names, tm, m, acc):
    import itertools
    fd = res.fd
    Mm = np.vstack([fd.get(n)[1][m] for n in names[:4]])
    pos = list(LAYOUT_SIGNS)
    best, best_s = None, -9
    for perm in itertools.permutations(pos):
        rs = np.array([LAYOUT_SIGNS[p][0] for p in perm]); ps = np.array([LAYOUT_SIGNS[p][1] for p in perm])
        s = 0
        for x, a in ((rs @ Mm, "p"), (ps @ Mm, "q")):
            y = np.interp(tm[m] + 0.025, fd.get(a)[0], acc[a])
            s += np.corrcoef(x, y)[0, 1]
        if s > best_s:
            best, best_s = perm, s
    return ", ".join(f"M{i + 1} {p}" for i, p in enumerate(best)) if best else None


# ---------------------------------------------------------------- vibration
def _vibration(res):
    fd, th = res.fd, res.cfg["thresholds"]
    if not fd.has("p"):
        return
    v = res.vibration
    tg, _ = fd.get("p")
    fs = 1 / np.median(np.diff(tg))
    v["gyro_rate_hz"] = float(fs)
    s = res.summary
    windows = {}
    if s.get("arm_t") is not None and s.get("takeoff_t") is not None and s["takeoff_t"] - s["arm_t"] > 1.5:
        windows["ground, motors idle"] = (s["arm_t"] + 0.3, s["takeoff_t"] - 0.3)
    nw = res.motors.get("normal_window")
    if nw:
        windows["normal flight"] = (nw[0] + 1.0, nw[1])
    for name, (a, b) in windows.items():
        out = {}
        for ax in ("p", "q", "r"):
            tt, x = fd.get(ax)
            m = (tt >= a) & (tt <= b)
            if m.sum() < 50:
                continue
            out[ax] = {"hf_rms_dps": _highpass_rms(tt[m], x[m], 15.0), "peak_hz": _psd_peak(tt[m], x[m], 15.0)[0]}
        v[name] = out
        if out:
            worst = max(o["hf_rms_dps"] for o in out.values())
            sev = "BAD" if worst > th["gyro_noise_bad_dps"] else "WARN" if worst > th["gyro_noise_warn_dps"] else "OK"
            pk = max(out.values(), key=lambda o: o["hf_rms_dps"])["peak_hz"]
            res.add("Vibration", sev, f"Gyro noise ({name})",
                    f"High-frequency (>15 Hz) gyro RMS: " + ", ".join(f"{k} {o['hf_rms_dps']:.1f}" for k, o in out.items()) +
                    f" deg/s; dominant frequency ~{pk:.0f} Hz. " +
                    ("High: balance props, check motor bearings, soften the autopilot damping mount, secure loose wires/arms."
                     if sev != "OK" else "Fine."), "high",
                    issue=None if sev == "OK" else ("vibration_ground" if name.startswith("ground") else "vibration"))


# ---------------------------------------------------------------- control (ADRC)
def _control(res):
    fd, cfg = res.fd, res.cfg
    c = res.control
    nw = res.motors.get("normal_window")
    for ax in ("rol", "pit", "yaw"):
        k5, k6 = f"adrc_{ax}_5", f"adrc_{ax}_6"
        if not (fd.has(k5) and fd.has(k6)):
            continue
        tt, u = fd.get(k6)
        _, bu = fd.get(k5)
        m = np.abs(u) > 0.3
        if nw:
            m &= (tt >= nw[0]) & (tt <= nw[1])
        if m.sum() > 30:
            ratio = bu[m] / u[m]
            c[f"{ax}_b0_est"] = float(np.median(ratio))
            c[f"{ax}_b0_iqr"] = (float(np.percentile(ratio, 25)), float(np.percentile(ratio, 75)))
        if nw:
            mm = (tt >= nw[0]) & (tt <= nw[1])
            c[f"{ax}_u_rms"] = float(np.sqrt(np.mean(u[mm] ** 2)))
            c[f"{ax}_u_peak_hz"] = _psd_peak(tt[mm], u[mm], 0.5, 30)[0]
    if "yaw_b0_est" in c:
        lo, hi = c["yaw_b0_iqr"]
        res.add("Tuning", "INFO", "ADRC b0 read back from the log",
                f"yaw b0 ~ {c['yaw_b0_est']:.2f} (spread {lo:.2f}..{hi:.2f}); "
                + (f"roll ~ {c.get('rol_b0_est', float('nan')):.1f}, pitch ~ {c.get('pit_b0_est', float('nan')):.1f} "
                   "(roll/pitch read-back is noisy - treat as approximate). " if "rol_b0_est" in c else "")
                + "Derived from ADRC element[5]/element[6]; yaw matched the GCS value -1.2 on the reference log.",
                "medium")
    # oscillation in attitude during normal flight
    if nw and fd.has("p"):
        for ax, nm in (("p", "roll"), ("q", "pitch"), ("r", "yaw")):
            tt, x = fd.get(ax)
            m = (tt >= nw[0] + 1) & (tt <= nw[1])
            if m.sum() < 200:
                continue
            band_lo = _highpass_rms(tt[m], _smooth(x[m], int(1 / np.median(np.diff(tt)) / 12) + 1), 1.0)  # 1..~6 Hz
            f_pk, _ = _psd_peak(tt[m], _smooth(x[m], 5), 0.7, 8.0)
            c[f"{nm}_osc_rms_dps"] = band_lo
            c[f"{nm}_osc_peak_hz"] = f_pk


# ---------------------------------------------------------------- attitude vs acceleration
def _attitude_consistency(res):
    """During the upset onset: did the measured acceleration follow the logged tilt?

    If yes, the tilt was physically real (not an attitude-estimator glitch).
    Only the onset is used: once speed builds up, drag dominates and the
    simple thrust-only model is no longer valid.
    """
    fd, up = res.fd, res.upset
    if not (fd.has("qw") and fd.has("accw_n") and up.get("t_start") is not None):
        return
    t = np.arange(up["t_start"] - 0.5, up["t_upset"] + 0.6, 0.01)
    R = _quat_R(fd, t)
    bz = -R[:, 2, :]
    hn, he = bz[0] / np.abs(bz[2]), bz[1] / np.abs(bz[2])     # tilt direction (per g)
    an = _smooth(np.interp(t, *fd.get("accw_n")), 10)
    ae = _smooth(np.interp(t, *fd.get("accw_e")), 10)
    pred = np.concatenate([hn, he]); meas = np.concatenate([an, ae])
    c = float(np.corrcoef(pred, meas)[0, 1])
    res.control["onset_tilt_vs_accel_corr"] = c
    if c > 0.5:
        res.add("Estimator", "INFO", "The tilt was real (not an estimator glitch)",
                f"During the dive the measured horizontal acceleration followed the logged tilt (corr {c:.2f}). "
                "So the model really tipped over; the attitude estimate was not the problem.", "medium", up["t_start"])
    else:
        res.add("Estimator", "WARN", "Tilt and acceleration disagree during the dive",
                f"corr {c:.2f}: the logged attitude may be wrong (IMU/estimator fault) - check IMU calibration and vibration.",
                "low", up["t_start"], issue="estimator_mismatch")


# ---------------------------------------------------------------- power
def _power(res):
    fd = res.fd
    if not fd.has("volt"):
        return
    t, v = fd.get("volt")
    p = res.power
    p["v_start"], p["v_min"], p["v_end"] = float(np.median(v[:5])), float(np.min(v)), float(np.median(v[-5:]))
    cells = res.cfg["airframe"].get("battery_cells") or int(round(p["v_start"] / 4.0))
    cells = max(1, cells)
    p["cells"] = cells
    p["t_v_min"] = float(t[np.argmin(v)])
    t_up = res.upset.get("t_start")
    vn = v[t < t_up] if t_up is not None and (t < t_up).sum() > 3 else v
    p["v_min_normal"] = float(vn.min())
    p["v_cell_min"] = p["v_min_normal"] / cells
    sev = "BAD" if p["v_cell_min"] < 3.3 else "WARN" if p["v_cell_min"] < 3.5 else "OK"
    res.add("Power", sev, "Battery voltage",
            f"{cells}S (guessed from voltage): start {p['v_start']:.2f} V ({p['v_start'] / cells:.2f} V/cell), "
            f"lowest in normal flight {p['v_min_normal']:.2f} V ({p['v_cell_min']:.2f} V/cell), end {p['v_end']:.2f} V.",
            issue=None if sev == "OK" else "battery_low")
    if t_up is not None and p["v_min"] < p["v_min_normal"] - 0.3:
        res.add("Power", "INFO", "Voltage collapsed during the crash",
                f"Down to {p['v_min']:.2f} V ({p['v_min'] / cells:.2f} V/cell) at t={p['t_v_min']:.1f}s while the motors ran at "
                "full power after the upset - an effect of the crash, not its cause.", "high", p["t_v_min"])


# ---------------------------------------------------------------- tuning advisor
def _tuning_advice(res):
    """Map wobble metrics to an ADRC b0 change of +/-0.25 or +/-0.5.

    ADRC: u = (u0 - z3) / b0. Larger |b0| -> smaller output -> softer.
    fast wobble in steady flight -> softer; slow swaying + overshoot -> stiffer.
    Direction can be flipped in config: tuning.higher_b0_means = 'stiffer'.
    """
    c, cfg = res.control, res.cfg["tuning"]
    wob = c.get("wobble", {})
    soft_dir = 1 if cfg.get("higher_b0_means", "softer") == "softer" else -1
    small, large = cfg["step_small"], cfg["step_large"]
    issues = {f.issue for f in res.findings if f.severity in ("WARN", "BAD")}
    mech_first = bool(issues & {"vibration", "vibration_ground", "wobble_braking", "weak_motor", "motor_failure",
                                "saturation_high", "thrust_reserve", "esc_desync"})
    over = c.get("braking_overshoot_ms")
    for axis, key, cur_key, b0_key in (("Roll", "roll", "roll_pitch_b0", "rol_b0_est"),
                                       ("Pitch", "pitch", "roll_pitch_b0", "pit_b0_est"),
                                       ("Yaw", "yaw", "yaw_b0", "yaw_b0_est")):
        o = wob.get(key)
        if not o:
            continue
        cur = cfg.get(cur_key) if cfg.get(cur_key) is not None else c.get(b0_key)
        lim = 5.0 if key != "yaw" else 6.0
        reasons = [f"fast wobble {o['fast_steady']:.1f} deg/s RMS (limit {lim:.0f})",
                   f"slow sway {o['slow_steady']:.1f} deg/s RMS"]
        action, step = "keep", 0.0
        if o["fast_steady"] > 2 * lim:
            action, step = "softer", large
        elif o["fast_steady"] > lim:
            action, step = "softer", small
        elif key != "yaw" and "sluggish" in issues | {f.issue for f in res.findings}:
            action, step = "stiffer", small
            reasons.append(f"overshoot after braking {over:.1f} m/s")
        if mech_first and action != "keep":
            reasons.append("fix the mechanical/propulsion findings first, then re-check")
        rec = None
        if action != "keep" and cur is not None:
            sign = soft_dir if action == "softer" else -soft_dir
            rec = math.copysign(abs(cur) + sign * step, cur)
        res.tuning.append({"axis": axis, "current_b0": cur, "action": action, "step": step,
                           "recommended_b0": rec, "reasons": reasons,
                           "confidence": "low" if (res.upset or mech_first) else "medium"})
    if res.upset:
        res.add("Tuning", "WARN", "Tuning advice is limited for this log",
                "The flight ended in an upset/crash; tuning numbers use only the part before the upset. "
                "Fix the root cause first, then tune with a clean hover + braking test flight.")
