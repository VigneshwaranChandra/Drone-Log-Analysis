"""Pro-level issue detectors. Each one adds Findings tagged with an issue id from knowledge.ISSUES.

All detectors are defensive: missing channels -> skip silently.
Thresholds were calibrated on simulated faults (tools/simulate_rlog.py) and the real sample log.
"""
from __future__ import annotations

import math

import numpy as np

from .core import G, LAYOUT_SIGNS, _runs, _smooth, _quat_R


def run(res):
    for det in (motor_pattern, motor_failure, saturation, power_loss, collision, hard_landing, esc_desync,
                gps_checks, wind_fit, wobble, hover_hold, battery_sag, log_gaps):
        try:
            det(res)
        except Exception as e:  # a broken detector must never kill the report
            res.add("Tool", "INFO", f"Detector {det.__name__} skipped", f"{type(e).__name__}: {e}", "high")


def _band(x, fs, lo, hi):
    """Crude band-pass with moving averages (no scipy needed): low-pass at hi, minus low-pass at lo."""
    a = _smooth(x, max(1, int(fs / hi)))
    b = _smooth(x, max(3, int(fs / lo)))
    return a - b


def _normal_mask(res, t):
    nw = res.motors.get("normal_window")
    if not nw:
        return np.zeros_like(t, bool)
    m = (t >= nw[0]) & (t <= nw[1])
    return m


# ------------------------------------------------------------------ motors
def motor_pattern(res):
    mo, cfg = res.motors, res.cfg
    if "mean_pwm" not in mo or len(mo["mean_pwm"]) != 4:
        return
    rng = cfg["airframe"]["pwm_max"] - cfg["airframe"]["pwm_min"]
    d = 100 * (np.array(mo["mean_pwm"]) - np.mean(mo["mean_pwm"])) / rng     # % of range
    lay = cfg["airframe"].get("motor_layout", {})
    names = mo["names"]
    pos = [lay.get(n, "?") for n in names]
    lab = [f"M{i + 1} ({p})" for i, p in enumerate(pos)]
    # 1) single motor standing out
    k = int(np.argmax(np.abs(d - np.median(d))))
    others = np.delete(d, k)
    stand = d[k] - others.mean()
    single = abs(stand) > 2.0 and np.std(others) < 0.35 * abs(stand)
    if single:
        sev = "BAD" if abs(stand) > 6 else "WARN"
        if stand > 0:
            res.add("Motors", sev, f"{lab[k]} works harder than the other three",
                    f"{lab[k]} mean command {stand:+.1f}% of range above the others (others agree within {np.std(others):.1f}%). "
                    "That motor/prop/ESC gives less thrust per command.", "medium", issue="weak_motor")
        elif abs(stand) > 3.0:
            res.add("Motors", "INFO", f"{lab[k]} works less than the other three",
                    f"{lab[k]} mean command {stand:+.1f}% vs others. Usually the OPPOSITE motor is weak or this "
                    "motor/prop gives more thrust (bigger/different prop).", "low", issue="weak_motor")
        if abs(stand) > 3.0 or stand > 0:
            return
    # 2) roll/pitch/yaw decomposition (needs layout)
    signs = [LAYOUT_SIGNS.get(p) for p in pos]
    if not all(signs):
        return
    rs = np.array([s[0] for s in signs], float); ps = np.array([s[1] for s in signs], float)
    ys = rs * ps          # diagonal pairs
    a_r, a_p, a_y = (rs @ d) / 2, (ps @ d) / 2, (ys @ d) / 2
    mo["pattern_pct"] = {"left_minus_right": a_r, "front_minus_rear": a_p, "diagonal": a_y}
    parts = []
    if abs(a_r) > 2.0:
        parts.append(f"{'left' if a_r > 0 else 'right'} side +{abs(a_r):.1f}%")
    if abs(a_p) > 2.0:
        parts.append(f"{'front' if a_p > 0 else 'rear'} +{abs(a_p):.1f}%")
    if parts:
        heavy = " and ".join(parts)
        res.add("Motors", "WARN" if max(abs(a_r), abs(a_p)) < 6 else "BAD", "Centre of gravity offset (or steady wind)",
                f"Motors on the {heavy} of range on average -> the model is heavy on that side. "
                f"Move battery/payload the other way. Mean PWM: " + ", ".join(f"{l} {m:.0f}" for l, m in zip(lab, mo['mean_pwm'])) + ".",
                "medium", issue="cg_offset")
    if abs(a_y) > 2.0:
        pair = [lab[i] for i in range(4) if ys[i] * a_y > 0]
        res.add("Motors", "WARN", "Yaw (diagonal) imbalance",
                f"Diagonal pair {' + '.join(pair)} works {abs(a_y):.1f}% harder to hold heading -> twisted arm / tilted motor "
                "or mismatched props.", "medium", issue="yaw_imbalance")
    if not parts and abs(a_y) <= 2.0:
        res.add("Motors", "OK", "Motor balance",
                "Mean PWM: " + ", ".join(f"{l} {m:.0f}" for l, m in zip(lab, mo["mean_pwm"])) +
                " - no CG, yaw or single-motor imbalance.")


def motor_failure(res):
    fd, cfg = res.fd, res.cfg
    names = fd.motor_names()
    if len(names) < 4:
        return
    t, _ = fd.get(names[0])
    M = np.vstack([np.interp(t, *fd.get(n)) for n in names])
    pmax = cfg["airframe"]["pwm_max"]
    air = np.interp(t, res.grid_t, res.grid["_air"]) > 0.5
    stop = (res.upset["t_upset"] + 0.3) if res.upset.get("t_upset") is not None else t[-1]
    fs = 1 / np.median(np.diff(t))
    for k in range(len(names)):
        others = np.delete(M, k, axis=0)
        pinned = air & (M[k] >= pmax - 5) & (others.mean(0) < pmax - 120) & (t < stop)
        for a, b in _runs(pinned):
            if (b - a) / fs >= 0.4:
                t0 = float(t[a])
                pos = cfg["airframe"].get("motor_layout", {}).get(names[k], "")
                before = res.upset.get("t_start") is not None and t0 <= res.upset["t_upset"] + 0.3
                res.add("Crash" if before else "Motors", "BAD",
                        f"M{k + 1} ({pos}) pinned at max while the others were lower",
                        f"From t={t0:.2f}s for {(b - a) / fs:.1f}s M{k + 1} was at full command while the mean of the others was "
                        f"{others[:, a:b].mean():.0f} us. The controller asked that corner for more thrust and did not get it -> "
                        f"thrust loss on M{k + 1} (prop/motor/ESC)" + (" BEFORE the upset - likely crash cause." if before else "."),
                        "high" if before else "medium", t0, issue="motor_failure")
                if before:
                    res.upset["cause_motor"] = k + 1
                break
        if res.upset.get("cause_motor"):
            break


def saturation(res):
    fd, cfg, th = res.fd, res.cfg, res.cfg["thresholds"]
    names = fd.motor_names()
    if not names:
        return
    t, g = res.grid_t, res.grid
    M = np.vstack([g[n] for n in names])
    m = _normal_mask(res, t)
    if m.sum() < 50:
        return
    hi = np.nanmax(M[:, m], 0) >= cfg["airframe"]["pwm_max"] - 5
    lo = np.nanmin(M[:, m], 0) <= cfg["airframe"]["pwm_idle"] + 5
    ph = g.get("_phase")
    for arr, lab, issue in ((hi, "maximum", "saturation_high"), (lo, "idle (minimum)", "saturation_low")):
        pct = 100 * arr.mean()
        if pct > th["saturation_warn_pct"]:
            where = ""
            if ph is not None:
                phs = ph[m][arr]
                if phs.size:
                    vals, cnt = np.unique(phs, return_counts=True)
                    order = np.argsort(-cnt)[:3]
                    where = " Mostly during: " + ", ".join(f"{vals[i]} ({100 * cnt[i] / phs.size:.0f}%)" for i in order) + "."
            res.add("Motors", "BAD" if pct > th["saturation_bad_pct"] else "WARN", f"Motors at {lab}",
                    f"At least one motor at {lab} for {pct:.1f}% of normal flight.{where}", "high", issue=issue)
    thr = res.motors.get("avg_throttle_pct")
    if thr is not None and thr > th["hover_throttle_high_pct"]:
        res.add("Motors", "WARN" if thr < 75 else "BAD", "Low thrust reserve",
                f"Average motor output {thr:.0f}% of range (target 45-60%).", "high", issue="thrust_reserve")


# ------------------------------------------------------------------ crash types
def power_loss(res):
    fd = res.fd
    if not fd.has("alt_rel") or not fd.motor_names():
        return
    ta, alt = fd.get("alt_rel")
    t_end = min(ta[-1], min(fd.get(n)[0][-1] for n in fd.motor_names()))
    a_end = float(np.interp(t_end, ta, alt))
    mm = float(np.mean([np.interp(t_end, *fd.get(n)) for n in fd.motor_names()]))
    if a_end > 2.0 and mm > res.cfg["airframe"]["pwm_idle"] + 50 and not res.upset.get("t_impact"):
        v = ""
        if fd.has("volt"):
            tv, vv = fd.get("volt")
            v = f" Last battery voltage {vv[-1]:.2f} V (min {vv.min():.2f} V)."
        res.add("Crash", "BAD", "Log stopped while flying (power loss / autopilot reset)",
                f"The last record (t={t_end:.1f}s) is at {a_end:.1f} m altitude with motors at {mm:.0f} us.{v}",
                "high", float(t_end), issue="power_loss")


def collision(res):
    up = res.upset
    if not up.get("t_impact") or "_tilt" not in res.grid:
        return
    t, g = res.grid_t, res.grid
    ti = up["t_impact"]
    m = (t > ti - 1.0) & (t < ti - 0.05)
    if m.any() and np.nanmax(g["_tilt"][m]) < 30 and np.nanmean(g["alt_rel"][m]) > 1.0:
        res.add("Crash", "BAD", "Impact while flying level - collision",
                f"Tilt stayed below 30 deg in the second before the {up['peak_accel_ms2'] / G:.0f} g impact at "
                f"{np.nanmean(g['alt_rel'][m]):.1f} m altitude, speed {np.nanmean(g['_hspeed'][m]):.1f} m/s.",
                "medium", ti, issue="collision")


def hard_landing(res):
    t, g = res.grid_t, res.grid
    if "vel_u" not in g:
        return
    air = g["_air"] > 0.5
    for a, b in _runs(air):
        if b >= len(t) - 2 or (b - a) < 100:
            continue
        if res.upset.get("t_start") is not None and t[b] > res.upset["t_start"]:
            continue
        w = slice(max(a, b - 50), b)
        vz = -np.nanmin(g["vel_u"][w])
        if vz > 2.0:
            res.add("Landing", "BAD" if vz > 3.0 else "WARN", "Hard landing",
                    f"Descending at {vz:.1f} m/s just before touchdown at t={t[b]:.1f}s (safe: < 1 m/s).",
                    "high", float(t[b]), issue="hard_landing")
        else:
            res.add("Landing", "OK", "Landing", f"Touchdown at {vz:.1f} m/s (t={t[b]:.1f}s).", "high", float(t[b]))


def esc_desync(res):
    fd, cfg = res.fd, res.cfg
    if not (fd.has("p") and fd.has("q")) or len(fd.motor_names()) != 4:
        return
    tp, p = fd.get("p"); _, q = fd.get("q")
    fs = 1 / np.median(np.diff(tp))
    pd = np.gradient(_smooth(p, int(fs / 25) + 1), tp)
    qd = np.gradient(_smooth(q, int(fs / 25) + 1), tp)
    acc = np.hypot(pd, qd)
    nw = res.motors.get("normal_window")
    if not nw:
        return
    m = (tp > nw[0] + 2) & (tp < nw[1])
    if m.sum() < 400:
        return
    base = np.percentile(acc[m], 99)
    thr = max(1200.0, 4 * np.percentile(acc[m], 90))
    peaks = np.flatnonzero(m & (acc > thr))
    events = []
    last = -1e9
    for i in peaks:
        if tp[i] - last > 0.3:
            events.append(i)
            last = tp[i]
    if len(events) < 3:
        return
    lay = cfg["airframe"].get("motor_layout", {})
    names = fd.motor_names()
    pos = {"front-right": (1, 1), "front-left": (1, -1), "rear-left": (-1, -1), "rear-right": (-1, 1)}
    votes = np.zeros(4)
    for i in events:
        # a motor at (x,y) losing thrust -> roll accel ~ +y, pitch accel ~ -x
        v = np.array([pd[i], qd[i]])
        best = max(range(4), key=lambda k: v @ np.array([pos.get(lay.get(names[k]), (0, 0))[1],
                                                          -pos.get(lay.get(names[k]), (0, 0))[0]]))
        votes[best] += 1
    k = int(np.argmax(votes))
    share = votes[k] / votes.sum()
    if share >= 0.6:
        res.add("Motors", "WARN", f"Short thrust drop-outs on M{k + 1} ({lay.get(names[k], '')})",
                f"{len(events)} sudden rotation kicks (> {thr:.0f} deg/s^2) not asked for by the controller; "
                f"{share * 100:.0f}% of them point at M{k + 1}. Typical of ESC desync or a loose motor wire. "
                "(Motor attribution assumes the configured layout is right.)", "medium",
                float(tp[events[0]]), issue="esc_desync")


# ------------------------------------------------------------------ control quality
def wobble(res):
    fd, g, t = res.fd, res.grid, res.grid_t
    if not (fd.has("p") and fd.has("q")):
        return
    tp, _ = fd.get("p")
    fs = 1 / np.median(np.diff(tp))
    nw = res.motors.get("normal_window")
    if not nw:
        return
    ph = g.get("_phase")
    win, step = 2.0, 0.5
    rows = []
    for ax in ("p", "q", "r"):
        tt, x = fd.get(ax)
        fast = _band(x, fs, 2.0, 10.0)
        slow = _band(x, fs, 0.3, 1.5)
        for c in np.arange(nw[0] + 1 + win / 2, nw[1] - win / 2, step):
            m = (tt >= c - win / 2) & (tt < c + win / 2)
            if m.sum() < 20:
                continue
            phase = ph[np.argmin(np.abs(t - c))] if ph is not None else "-"
            rows.append((ax, c, phase, _osc_rms(fast[m]), float(np.sqrt(np.mean(slow[m] ** 2)))))
    if not rows:
        return
    out = {}
    for ax, nm in (("p", "roll"), ("q", "pitch"), ("r", "yaw")):
        R = [r for r in rows if r[0] == ax]
        steady = [r for r in R if r[2] in ("hover", "cruise", "turn", "yaw-turn", "climb", "descent")]
        trans = [r for r in R if r[2] in ("braking", "accelerating")]
        f_st = np.median([r[3] for r in steady]) if steady else np.nan
        f_tr = np.percentile([r[3] for r in trans], 75) if trans else np.nan
        s_st = np.median([r[4] for r in steady]) if steady else np.nan
        frac = np.mean([r[3] > (5.0 if nm != "yaw" else 6.0) for r in R])
        out[nm] = {"fast_steady": float(f_st), "fast_transition": float(f_tr), "slow_steady": float(s_st),
                   "fast_frac": float(frac)}
        # dominant frequency in 1-12 Hz over the normal window
        tt, x = fd.get(ax)
        m = (tt >= nw[0] + 1) & (tt <= nw[1])
        try:
            from scipy.signal import welch
            f, pxx = welch(_smooth(x[m], max(1, int(fs / 15))), fs=fs, nperseg=min(1024, m.sum()))
            mm = (f >= 1) & (f <= 12)
            out[nm]["peak_hz"] = float(f[mm][np.argmax(pxx[mm])])
        except Exception:
            out[nm]["peak_hz"] = float("nan")
    res.control["wobble"] = out
    for nm, o in out.items():
        lim = 5.0 if nm != "yaw" else 6.0
        if o["fast_steady"] > lim:
            res.add("Control", "BAD" if o["fast_steady"] > 2 * lim else "WARN", f"Fast wobble on {nm}",
                    f"{nm} rate wobble {o['fast_steady']:.1f} deg/s RMS (2-10 Hz) in steady flight, peak ~{o['peak_hz']:.1f} Hz, "
                    f"present {o['fast_frac'] * 100:.0f}% of the time -> controller too aggressive or mechanical play.",
                    "medium", issue="wobble_fast")
        elif np.isfinite(o["fast_transition"]) and o["fast_transition"] > lim and o["fast_steady"] < 0.5 * lim:
            res.add("Control", "WARN", f"Wobble on {nm} mainly when braking/accelerating",
                    f"{nm} wobble {o['fast_transition']:.1f} deg/s RMS during braking/accelerating vs {o['fast_steady']:.1f} in steady "
                    "flight -> check mechanics (arms, wires near props, autopilot mount) before tuning.",
                    "medium", issue="wobble_braking")
    # sluggish: no fast wobble anywhere, but the model overshoots after stopping
    over = _braking_overshoot(res)
    res.control["braking_overshoot_ms"] = over
    calm = all(o["fast_steady"] < 3.0 for k, o in out.items() if k != "yaw")
    disturbed = any(f.issue in ("gps_glitch", "wind", "motor_failure", "power_loss") for f in res.findings)
    if over is not None and over > 2.5 and calm and not disturbed:
        res.add("Control", "INFO", "Slow to stop - overshoot after braking",
                f"After braking the model overshot by up to {over:.1f} m/s in the opposite direction, with no fast wobble. "
                "Either the attitude control is soft (b0 high) or the position/brake settings in the GCS are soft. "
                "Confirm by feel before changing b0.", "low", issue="sluggish")


def _osc_rms(x, min_cycles=3):
    """RMS of a band-passed signal, but only if it really oscillates: at least `min_cycles`
    full cycles whose peaks reach 40% of the window's maximum. A single manoeuvre transient
    (one push) returns a heavily reduced value."""
    if x.size < 10:
        return 0.0
    rms = float(np.sqrt(np.mean(x ** 2)))
    amp = np.max(np.abs(x))
    if amp <= 0:
        return 0.0
    big = np.abs(x) > 0.4 * amp
    s = np.sign(x)
    # count sign changes between big excursions
    idx = np.flatnonzero(big)
    flips = np.count_nonzero(np.diff(s[idx]) != 0) if idx.size > 1 else 0
    cycles = flips / 2.0
    return rms if cycles >= min_cycles else rms * cycles / min_cycles * 0.5


def _braking_overshoot(res):
    t, g = res.grid_t, res.grid
    if "_hspeed" not in g:
        return None
    worst = 0.0
    found = False
    for s in res.segments:
        if s["phase"] != "braking" or s["speed"] < 3:
            continue
        a = np.searchsorted(t, s["t0"]); b = np.searchsorted(t, s["t1"])
        if b - a < 10:
            continue
        v0 = np.array([np.nanmean(g["vel_n"][a:a + 10]), np.nanmean(g["vel_e"][a:a + 10])])
        if np.linalg.norm(v0) < 2:
            continue
        u = v0 / np.linalg.norm(v0)
        w = slice(b, min(len(t), b + 150))
        along = g["vel_n"][w] * u[0] + g["vel_e"][w] * u[1]
        if along.size:
            found = True
            worst = max(worst, float(-np.nanmin(along)))
    return worst if found else None


# ------------------------------------------------------------------ navigation
def gps_checks(res):
    fd = res.fd
    nw = res.motors.get("normal_window")
    if not nw:
        return
    if fd.has("gps_sats"):
        tt, s = fd.get("gps_sats")
        m = (tt >= nw[0]) & (tt <= nw[1])
        if m.any() and s[m].min() < 8:
            res.add("GPS", "WARN", "Low satellite count in flight", f"Minimum {s[m].min():.0f} satellites.", "high",
                    float(tt[m][np.argmin(s[m])]), issue="gps_weak")
    if fd.has("gps_fix"):
        tt, f = fd.get("gps_fix")
        m = (tt >= nw[0]) & (tt <= nw[1])
        if m.any() and f[m].min() < 3:
            res.add("GPS", "BAD", "GPS lost 3D fix in flight", f"Fix type dropped to {f[m].min():.0f}.", "high",
                    float(tt[m][np.argmin(f[m])]), issue="gps_weak")
    if not (fd.has("pos_n") and fd.has("vel_n")):
        return
    t = np.arange(nw[0], nw[1], 0.05)
    if len(t) < 40:
        return
    jumps = []
    for a in ("n", "e"):
        p = np.interp(t, *fd.get(f"pos_{a}"))
        v = np.interp(t, *fd.get(f"vel_{a}"))
        dp = p[20:] - p[:-20]
        iv = np.convolve(v, np.ones(20) * 0.05, "valid")[1:]
        n = min(len(dp), len(iv))
        jumps.append(np.abs(dp[:n] - iv[:n]))
    j = np.hypot(*jumps)
    res.control["gps_jump_max_m"] = float(j.max())
    if j.max() > 3.0:
        i = int(np.argmax(j))
        res.add("GPS", "BAD" if j.max() > 6 else "WARN", "Position jump (GPS glitch)",
                f"Position moved {j.max():.1f} m more than the velocity allows within 1 s around t={t[i]:.1f}s.",
                "high", float(t[i]), issue="gps_glitch")


def hover_hold(res):
    t, g = res.grid_t, res.grid
    for s in res.segments:
        if s["phase"] != "hover" or s["t1"] - s["t0"] < 5:
            continue
        m = (t >= s["t0"] + 1) & (t <= s["t1"] - 0.5)
        if m.sum() < 100 or "pos_n" not in g:
            continue
        pn, pe = g["pos_n"][m], g["pos_e"][m]
        spread = float(np.nanmax(np.hypot(pn - np.nanmean(pn), pe - np.nanmean(pe))))
        if spread > 1.5:
            res.add("Navigation", "WARN", "Position hold drift in hover",
                    f"Hover {s['t0']:.0f}-{s['t1']:.0f}s wandered up to {spread:.1f} m from its centre.", "medium",
                    s["t0"], issue="position_drift")
        if "alt_rel" in g:
            alt = g["alt_rel"][m]
            ar = float(np.nanmax(alt) - np.nanmin(alt))
            if ar > 1.0:
                res.add("Navigation", "WARN", "Altitude hold variation in hover",
                        f"Hover {s['t0']:.0f}-{s['t1']:.0f}s altitude varied by {ar:.1f} m.", "medium", s["t0"],
                        issue="altitude_hold")


def wind_fit(res):
    """Fit wind (N,E) and drag k from steady samples: a_meas - a_thrust = -k |v-w| (v-w)."""
    fd, t, g = res.fd, res.grid_t, res.grid
    if not (fd.has("qw") and fd.has("accw_n") and "_hspeed" in g):
        return
    m = _normal_mask(res, t) & (g["_air"] > 0.5) & (g["_tilt"] < 35)
    if m.sum() < 500:
        return
    tt = t[m]
    R = _quat_R(fd, tt)
    bz = -R[:, 2, :]
    an = _smooth(np.interp(tt, *fd.get("accw_n")), 25)
    ae = _smooth(np.interp(tt, *fd.get("accw_e")), 25)
    au = _smooth(np.interp(tt, *fd.get("accw_u")), 25)
    f = (G + au) / np.abs(bz[2])
    dn, de = an - f * bz[0], ae - f * bz[1]        # drag acceleration
    vn, ve = g["vel_n"][m], g["vel_e"][m]
    steady = np.hypot(an, ae) < 1.0
    if steady.sum() < 300:
        return
    vn, ve, dn, de = vn[steady], ve[steady], dn[steady], de[steady]
    course = np.degrees(np.arctan2(ve, vn))
    fast = np.hypot(vn, ve) > 2
    span = 0.0
    if fast.sum() > 50:
        h = np.histogram(course[fast] % 360, bins=12, range=(0, 360))[0]
        span = 30.0 * (h > fast.sum() * 0.03).sum()
    try:
        from scipy.optimize import least_squares
    except Exception:
        return
    def resid(x):
        wn, we, k = x
        an_, ae_ = vn - wn, ve - we
        sp = np.hypot(an_, ae_)
        return np.concatenate([dn + k * sp * an_, de + k * sp * ae_])
    if span >= 150:
        sol = least_squares(resid, [0.0, 0.0, 0.01], bounds=([-25, -25, 1e-4], [25, 25, 1.0]))
        wn, we, k = sol.x
        src = "drag fit over legs in different directions"
        conf = "medium"
    else:
        k = res.cfg["airframe"].get("cd_area_m2", 0.06) * 0.5 * 1.225 / res.cfg["airframe"].get("mass_kg", 2.0)
        hv = np.hypot(vn, ve) < 1.0
        if hv.sum() < 100:
            return
        sol = least_squares(lambda x: resid([x[0], x[1], k])[np.concatenate([hv, hv])], [0.0, 0.0])
        wn, we = sol.x
        src = "hover lean with assumed drag (mass/drag area from config)"
        conf = "low"
    spd = float(math.hypot(wn, we))
    frm = float((math.degrees(math.atan2(-we, -wn))) % 360)     # wind FROM direction
    res.wind.update({"speed_ms": spd, "from_deg": frm, "k_drag": float(k), "method": src})
    sev = "WARN" if spd > 6 else "INFO"
    res.add("Wind", sev, "Wind estimate",
            f"~{spd:.1f} m/s from {frm:.0f} deg ({_compass(frm)}) ({src}).",
            conf, issue="wind" if spd > 6 else None)


def _compass(d):
    return ["N", "NE", "E", "SE", "S", "SW", "W", "NW"][int(((d + 22.5) % 360) // 45)]


# ------------------------------------------------------------------ power / data
def battery_sag(res):
    fd = res.fd
    if not fd.has("volt") or not fd.motor_names():
        return
    tv, v = fd.get("volt")
    nw = res.motors.get("normal_window")
    if not nw:
        return
    m = (tv >= nw[0]) & (tv <= nw[1])
    if m.sum() < 20:
        return
    names = fd.motor_names()
    rng = res.cfg["airframe"]["pwm_max"] - res.cfg["airframe"]["pwm_min"]
    thr = np.mean([np.interp(tv[m], *fd.get(n)) for n in names], axis=0)
    thr = 100 * (thr - res.cfg["airframe"]["pwm_min"]) / rng
    if np.ptp(thr) < 10:
        return
    X = np.c_[thr, tv[m] - tv[m][0], np.ones(m.sum())]      # remove slow discharge trend
    c = np.linalg.lstsq(X, v[m], rcond=None)[0]
    cells = res.power.get("cells", 6)
    sag = -c[0] * 10 / cells                                # V per cell per 10% throttle
    res.power["sag_v_per_cell_per_10pct"] = float(sag)
    if sag > 0.06:
        res.add("Power", "BAD" if sag > 0.10 else "WARN", "Battery sags strongly under load",
                f"Voltage drops {sag * 1000:.0f} mV per cell for every +10% motor output (healthy pack < ~40 mV). "
                "High internal resistance or a bad connector.", "medium", issue="battery_sag")


def log_gaps(res):
    fd = res.fd
    key = "roll" if fd.has("roll") else None
    if not key:
        return
    t, _ = fd.get(key)
    dt = np.diff(t)
    gaps = np.flatnonzero(dt > max(0.1, 10 * np.median(dt)))
    if gaps.size:
        res.add("Data", "WARN", "Gaps in the log",
                f"{gaps.size} gap(s) in attitude data, longest {dt[gaps].max():.2f}s at t={t[gaps[np.argmax(dt[gaps])]]:.1f}s.",
                "high", issue="log_gaps")
