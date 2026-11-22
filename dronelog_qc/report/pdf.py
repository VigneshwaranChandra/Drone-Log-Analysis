"""PDF report: charts (matplotlib) + tables (reportlab)."""
from __future__ import annotations

import io
import os
import datetime as _dt

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from reportlab.lib import colors
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle, getSampleStyleSheet
from reportlab.lib.units import mm
from reportlab.platypus import (Image, KeepTogether, PageBreak, Paragraph, SimpleDocTemplate, Spacer,
                                Table, TableStyle)

from .. import __version__
from ..knowledge import ISSUES, POSTCRASH, PREFLIGHT, WHERE

SEV_COLOR = {"BAD": colors.HexColor("#c62828"), "WARN": colors.HexColor("#ef6c00"),
             "INFO": colors.HexColor("#1565c0"), "OK": colors.HexColor("#2e7d32")}
PHASE_COLOR = {"ground": "#bdbdbd", "hover": "#81c784", "cruise": "#64b5f6", "turn": "#ba68c8",
               "yaw-turn": "#ce93d8", "accelerating": "#4dd0e1", "braking": "#ffb74d", "climb": "#aed581",
               "descent": "#a1887f", "UPSET": "#e53935"}
MOTOR_COLORS = ["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728", "#9467bd", "#8c564b", "#e377c2", "#7f7f7f"]

ss = getSampleStyleSheet()
H1 = ParagraphStyle("h1", parent=ss["Heading1"], fontSize=16, spaceAfter=6)
H2 = ParagraphStyle("h2", parent=ss["Heading2"], fontSize=12.5, spaceBefore=8, spaceAfter=4)
BODY = ParagraphStyle("b", parent=ss["BodyText"], fontSize=8.8, leading=11.5)
SMALL = ParagraphStyle("s", parent=BODY, fontSize=7.6, leading=9.5, textColor=colors.HexColor("#444444"))
CELL = ParagraphStyle("c", parent=BODY, fontSize=7.8, leading=9.6)


def _fig_to_img(fig, width_mm=180):
    buf = io.BytesIO()
    fig.savefig(buf, format="png", dpi=150, bbox_inches="tight")
    plt.close(fig)
    buf.seek(0)
    w, h = fig.get_size_inches()
    width = width_mm * mm
    return Image(buf, width=width, height=width * h / w)


def _shade(ax, res):
    for s in res.segments:
        if s["phase"] in ("ground",):
            continue
        ax.axvspan(s["t0"], s["t1"], color=PHASE_COLOR.get(s["phase"], "#eeeeee"), alpha=0.18, lw=0)
    up = res.upset
    for key, c, lab in (("t_start", "#e53935", "upset start"), ("t_impact", "k", "impact")):
        if up.get(key) is not None:
            ax.axvline(up[key], color=c, ls="--", lw=1)


def _tbl(rows, widths, header=True, zebra=True, extra=None):
    data = [[Paragraph(str(c), CELL) if not isinstance(c, Paragraph) else c for c in r] for r in rows]
    t = Table(data, colWidths=[w * mm for w in widths], repeatRows=1 if header else 0)
    st = [("GRID", (0, 0), (-1, -1), 0.25, colors.HexColor("#bbbbbb")),
          ("VALIGN", (0, 0), (-1, -1), "TOP"),
          ("TOPPADDING", (0, 0), (-1, -1), 2), ("BOTTOMPADDING", (0, 0), (-1, -1), 2)]
    if header:
        st += [("BACKGROUND", (0, 0), (-1, 0), colors.HexColor("#263238")),
               ("TEXTCOLOR", (0, 0), (-1, 0), colors.white)]
    if zebra:
        for i in range(1 if header else 0, len(rows)):
            if i % 2 == 0:
                st.append(("BACKGROUND", (0, i), (-1, i), colors.HexColor("#f5f5f5")))
    t.setStyle(TableStyle(st + (extra or [])))
    if header:
        for c in range(len(rows[0])):
            data[0][c] = Paragraph(f"<font color='white'><b>{rows[0][c]}</b></font>", CELL)
    return t


# ------------------------------------------------------------------ charts
def chart_overview(res):
    t, g = res.grid_t, res.grid
    fig, ax = plt.subplots(4, 1, figsize=(10, 8.2), sharex=True)
    if "alt_rel" in g:
        ax[0].plot(t, g["alt_rel"], "k", lw=1.2)
    ax[0].set_ylabel("Altitude [m]")
    if "_hspeed" in g:
        ax[1].plot(t, g["_hspeed"], color="#1565c0", lw=1, label="horizontal")
    if "vel_u" in g:
        ax[1].plot(t, g["vel_u"], color="#2e7d32", lw=1, label="vertical (up +)")
    ax[1].set_ylabel("Speed [m/s]"); ax[1].legend(fontsize=7, loc="upper left")
    for k, c in (("roll", "#d32f2f"), ("pitch", "#1976d2")):
        if k in g:
            ax[2].plot(t, g[k], color=c, lw=0.9, label=k)
    ax[2].set_ylabel("Attitude [deg]"); ax[2].legend(fontsize=7, loc="upper left")
    names = res.fd.motor_names()
    for i, n in enumerate(names):
        ax[3].plot(t, g[n], lw=0.8, color=MOTOR_COLORS[i % 8], label=f"M{i + 1}")
    ax[3].set_ylabel("Motor PWM [us]"); ax[3].legend(fontsize=7, ncol=4, loc="upper left")
    ax[3].set_xlabel("Time from log start [s]")
    for a in ax:
        _shade(a, res); a.grid(alpha=0.3)
    handles = [plt.Rectangle((0, 0), 1, 1, color=c, alpha=0.4) for p, c in PHASE_COLOR.items()
               if any(s["phase"] == p for s in res.segments)]
    labels = [p for p in PHASE_COLOR if any(s["phase"] == p for s in res.segments)]
    fig.legend(handles, labels, loc="upper center", ncol=len(labels), fontsize=7, frameon=False)
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    return fig


def chart_map(res):
    t, g = res.grid_t, res.grid
    if not ("pos_n" in g and "pos_e" in g):
        return None
    fig, ax = plt.subplots(1, 2, figsize=(10, 4.3))
    ph = g.get("_phase")
    e, n = g["pos_e"], g["pos_n"]
    for p, c in PHASE_COLOR.items():
        m = ph == p if ph is not None else np.ones_like(e, bool)
        if np.any(m):
            ax[0].scatter(e[m], n[m], s=2, color=c, label=p)
    ax[0].plot(e[0], n[0], "g^", ms=9, label="start")
    up = res.upset
    if up.get("t_start") is not None:
        i = np.argmin(np.abs(t - up["t_start"])); ax[0].plot(e[i], n[i], "rx", ms=10, mew=2, label="upset start")
    if up.get("t_impact") is not None:
        i = np.argmin(np.abs(t - up["t_impact"])); ax[0].plot(e[i], n[i], "k*", ms=12, label="impact")
    ax[0].set_aspect("equal", "datalim"); ax[0].set_xlabel("East [m]"); ax[0].set_ylabel("North [m]")
    ax[0].set_title("Ground track (top view)", fontsize=9); ax[0].legend(fontsize=6, markerscale=2); ax[0].grid(alpha=0.3)
    if "alt_rel" in g:
        d = np.concatenate([[0], np.cumsum(np.hypot(np.diff(np.nan_to_num(e)), np.diff(np.nan_to_num(n))))])
        ax[1].scatter(d, g["alt_rel"], s=2, c=t, cmap="viridis")
        ax[1].set_xlabel("Distance flown [m]"); ax[1].set_ylabel("Altitude [m]")
        ax[1].set_title("Side view (colour = time)", fontsize=9); ax[1].grid(alpha=0.3)
    fig.tight_layout()
    return fig


def chart_upset(res):
    up = res.upset
    if up.get("t_start") is None:
        return None
    fd = res.fd
    a, b = up["t_start"] - 3.0, (up.get("t_impact") or up["t_upset"] + 3) + 1.0
    fig, ax = plt.subplots(4, 1, figsize=(10, 8), sharex=True)
    for k, c in (("roll", "#d32f2f"), ("pitch", "#1976d2")):
        tt, v = fd.get(k); m = (tt >= a) & (tt <= b); ax[0].plot(tt[m], v[m], color=c, label=k)
    ax[0].set_ylabel("deg"); ax[0].legend(fontsize=7)
    for i, n in enumerate(fd.motor_names()):
        tt, v = fd.get(n); m = (tt >= a) & (tt <= b)
        ax[1].plot(tt[m], v[m], ".-", ms=3, lw=0.9, color=MOTOR_COLORS[i], label=f"M{i + 1}")
    ax[1].axhline(res.cfg["airframe"]["pwm_max"], color="k", ls=":", lw=0.8)
    ax[1].set_ylabel("PWM"); ax[1].legend(fontsize=7, ncol=4)
    for k, c in (("p", "#d32f2f"), ("q", "#1976d2"), ("r", "#388e3c")):
        if fd.has(k):
            tt, v = fd.get(k); m = (tt >= a) & (tt <= b); ax[2].plot(tt[m], v[m], color=c, lw=0.6, label=k)
    ax[2].set_ylabel("rate [deg/s]"); ax[2].legend(fontsize=7)
    if fd.has("alt_rel"):
        tt, v = fd.get("alt_rel"); m = (tt >= a) & (tt <= b); ax[3].plot(tt[m], v[m], "k", label="altitude [m]")
    if fd.has("accw_n"):
        tk = fd.get("accw_n")[0]
        mag = np.sqrt(sum(fd.get(k)[1] ** 2 for k in ("accw_n", "accw_e", "accw_u")))
        m = (tk >= a) & (tk <= b)
        ax3 = ax[3].twinx(); ax3.plot(tk[m], mag[m] / 9.81, color="#ef6c00", lw=0.6); ax3.set_ylabel("accel [g]", color="#ef6c00")
    ax[3].legend(fontsize=7, loc="upper right"); ax[3].set_xlabel("Time [s]")
    for x in ax:
        x.grid(alpha=0.3)
        for key, c in (("t_start", "#e53935"), ("t_upset", "#b71c1c"), ("first_saturation_t", "#6a1b9a"), ("t_impact", "k")):
            if up.get(key) is not None:
                x.axvline(up[key], color=c, ls="--", lw=0.9)
    ax[0].set_title("Upset zoom  (red dashed = upset start / 60 deg, purple = first motor saturation, black = impact)", fontsize=8.5)
    fig.tight_layout()
    return fig


def chart_motor_balance(res):
    mo = res.motors
    if "mean_pwm" not in mo:
        return None
    lay = res.cfg["airframe"].get("motor_layout", {})
    fig, ax = plt.subplots(1, 2, figsize=(10, 3.4))
    names = [f"M{i + 1}\n{lay.get(n, '')}" for i, n in enumerate(mo["names"])]
    ax[0].bar(names, mo["mean_pwm"], color=MOTOR_COLORS[:len(names)])
    lo = min(mo["mean_pwm"]) - 60
    ax[0].set_ylim(lo, max(mo["mean_pwm"]) + 40); ax[0].set_ylabel("Mean PWM (normal flight)")
    for i, v in enumerate(mo["mean_pwm"]):
        ax[0].text(i, v + 5, f"{v:.0f}", ha="center", fontsize=8)
    ax[0].set_title("Motor balance", fontsize=9); ax[0].grid(alpha=0.3, axis="y")
    ax[1].bar(names, mo["noise_pwm"], color=MOTOR_COLORS[:len(names)])
    ax[1].set_ylabel("PWM noise >2 Hz [us RMS]"); ax[1].set_title("Motor command noise (high = aggressive / vibration)", fontsize=9)
    ax[1].grid(alpha=0.3, axis="y")
    fig.tight_layout()
    return fig


def chart_vibration(res):
    fd = res.fd
    if not fd.has("p"):
        return None
    try:
        from scipy.signal import welch
    except Exception:
        return None
    fig, ax = plt.subplots(1, 1, figsize=(10, 3.3))
    s, nw = res.summary, res.motors.get("normal_window")
    wins = []
    if s.get("arm_t") is not None and s.get("takeoff_t") is not None and s["takeoff_t"] - s["arm_t"] > 1.5:
        wins.append(("ground idle", s["arm_t"] + 0.3, s["takeoff_t"] - 0.3, ":"))
    if nw:
        wins.append(("normal flight", nw[0] + 1, nw[1], "-"))
    for lab, a, b, ls in wins:
        for k, c in (("p", "#d32f2f"), ("q", "#1976d2"), ("r", "#388e3c")):
            tt, v = fd.get(k); m = (tt >= a) & (tt <= b)
            if m.sum() < 128:
                continue
            fs = 1 / np.median(np.diff(tt))
            f, pxx = welch(v[m] - v[m].mean(), fs=fs, nperseg=min(512, m.sum()))
            ax.semilogy(f, pxx, ls=ls, color=c, lw=0.9, label=f"{k} {lab}")
    ax.set_xlabel("Frequency [Hz]"); ax.set_ylabel("Gyro PSD [(deg/s)^2/Hz]")
    ax.set_title("Gyro vibration spectrum (peaks = prop/motor vibration; low-frequency hump = control wobble)", fontsize=9)
    ax.legend(fontsize=7, ncol=2); ax.grid(alpha=0.3, which="both")
    fig.tight_layout()
    return fig


def chart_power(res):
    fd = res.fd
    if not fd.has("volt"):
        return None
    fig, ax = plt.subplots(1, 1, figsize=(10, 2.6))
    tt, v = fd.get("volt"); ax.plot(tt, v, "k", lw=1)
    ax.set_ylabel("Battery [V]"); ax.set_xlabel("Time [s]"); ax.grid(alpha=0.3); _shade(ax, res)
    if res.fd.motor_names():
        ax2 = ax.twinx()
        t, g = res.grid_t, res.grid
        mm = np.nanmean(np.vstack([g[n] for n in res.fd.motor_names()]), axis=0)
        ax2.plot(t, mm, color="#ef6c00", lw=0.6); ax2.set_ylabel("mean motor PWM", color="#ef6c00")
    fig.tight_layout()
    return fig


# ------------------------------------------------------------------ document
def _fmt(v, nd=1, unit=""):
    if v is None:
        return "-"
    if isinstance(v, float):
        return f"{v:.{nd}f}{unit}"
    return f"{v}{unit}"


def build_pdf(res, out_path, log_name=None):
    fd, s = res.fd, res.summary
    doc = SimpleDocTemplate(out_path, pagesize=A4, leftMargin=14 * mm, rightMargin=14 * mm,
                            topMargin=12 * mm, bottomMargin=12 * mm,
                            title="Drone Flight Log QC Report", author="dronelog_qc")
    el = []
    name = log_name or os.path.basename(fd.source)
    el.append(Paragraph("Drone Flight Log QC Report", H1))
    el.append(Paragraph(f"<b>Log:</b> {name} &nbsp; <b>Format:</b> {fd.meta.get('format', '-')} &nbsp; "
                        f"<b>Header:</b> {fd.meta.get('file_header', '-')} &nbsp; "
                        f"<b>Generated:</b> {_dt.datetime.now():%Y-%m-%d %H:%M} &nbsp; <b>Tool:</b> v{__version__}", SMALL))
    el.append(Spacer(1, 4))

    # verdict
    nbad = sum(f.severity == "BAD" for f in res.findings)
    nwarn = sum(f.severity == "WARN" for f in res.findings)
    verdict, vc = (("FAIL - do not fly until the BAD items are fixed", "BAD") if nbad else
                   ("PASS WITH WARNINGS", "WARN") if nwarn else ("PASS", "OK"))
    vt = Table([[Paragraph(f"<font color='white' size=12><b>QC VERDICT: {verdict}</b></font>", BODY)]],
               colWidths=[182 * mm])
    vt.setStyle(TableStyle([("BACKGROUND", (0, 0), (-1, -1), SEV_COLOR[vc]), ("TOPPADDING", (0, 0), (-1, -1), 6),
                            ("BOTTOMPADDING", (0, 0), (-1, -1), 6)]))
    el.append(vt)
    el.append(Spacer(1, 5))

    # summary table
    rows = [["Item", "Value", "Item", "Value"],
            ["Log length", _fmt(s.get("log_duration_s"), 1, " s"), "Air time", _fmt(s.get("air_time_s"), 1, " s")],
            ["Armed at", _fmt(s.get("arm_t"), 1, " s"), "Take-off at", _fmt(s.get("takeoff_t"), 1, " s")],
            ["Max altitude", _fmt(s.get("max_alt_m"), 1, " m"), "Max horizontal speed", _fmt(s.get("max_hspeed_ms"), 1, " m/s")],
            ["Max climb / descent", f"{_fmt(s.get('max_climb_ms'), 1)} / {_fmt(s.get('max_descent_ms'), 1)} m/s",
             "Distance flown", _fmt(s.get("distance_m"), 0, " m")],
            ["Location", s.get("location", "-"), "Min GPS satellites", _fmt(s.get("gps_sats_min"))],
            ["Autopilot uptime at log start", _fmt(s.get("boot_time_at_log_start_s"), 1, " s"),
             "Avg motor output", _fmt(res.motors.get("avg_throttle_pct"), 0, " %")]]
    el.append(_tbl(rows, [40, 51, 40, 51]))
    el.append(Paragraph("All times in this report are seconds from the first record in the log. "
                        "The autopilot only starts logging some time after power-on (see 'uptime at log start').", SMALL))

    # findings
    el.append(Paragraph("Findings (most serious first)", H2))
    rows = [["Level", "Area", "Finding", "Evidence / what to do", "Conf."]]
    extra = []
    for i, f in enumerate(res.findings, start=1):
        rows.append([f.severity, f.area, f.title + (f" (t={f.t:.1f}s)" if f.t is not None else ""), f.detail, f.confidence])
        extra.append(("TEXTCOLOR", (0, i), (0, i), SEV_COLOR.get(f.severity, colors.black)))
    el.append(_tbl(rows, [13, 17, 38, 100, 14], extra=extra))
    el.extend(where_section(res))
    el.extend(action_plan(res))

    # upset narrative
    if res.upset.get("t_start") is not None:
        el.append(PageBreak())
        el.append(Paragraph("Crash / upset reconstruction", H2))
        el.append(Paragraph(_upset_story(res), BODY))
        f = chart_upset(res)
        if f:
            el.append(_fig_to_img(f))

    el.append(PageBreak())
    el.append(Paragraph("Flight overview", H2))
    el.append(_fig_to_img(chart_overview(res)))
    f = chart_map(res)
    if f:
        el.append(_fig_to_img(f))

    # movement timeline
    el.append(Paragraph("What the model was doing (movement timeline)", H2))
    rows = [["From [s]", "To [s]", "Phase", "Speed [m/s]", "Vert. [m/s]", "Alt [m]", "Course / heading [deg]", "Max tilt"]]
    for sg in res.segments:
        rows.append([f"{sg['t0']:.1f}", f"{sg['t1']:.1f}", sg["phase"], f"{sg['speed']:.1f}", f"{sg['vz']:+.1f}",
                     _fmt(sg.get("alt"), 1), f"{_fmt(sg.get('course'), 0)} / {_fmt(sg.get('heading'), 0)}",
                     f"{sg['max_tilt']:.0f}"])
    el.append(_tbl(rows, [17, 17, 25, 20, 20, 18, 40, 18]))
    el.append(Paragraph("Course = direction of travel over ground, heading = where the nose points (0 = North, 90 = East). "
                        "A big difference between them means the model was flying sideways or being pushed by wind.", SMALL))

    # motors
    el.append(PageBreak())
    el.append(Paragraph("Motors", H2))
    mo = res.motors
    lc = mo.get("layout_check", {})
    el.append(Paragraph(f"Configured layout: " + ", ".join(f"M{i + 1} = {res.cfg['airframe']['motor_layout'].get(n, '?')}"
                                                          for i, n in enumerate(mo.get("names", []))) +
                        (f". <b>Physics check:</b> {lc.get('text', '')}" if lc else ""), BODY))
    f = chart_motor_balance(res)
    if f:
        el.append(_fig_to_img(f))
    if mo.get("mean_pwm"):
        el.append(Paragraph(f"Normal-flight window used: {mo['normal_window'][0]:.1f}-{mo['normal_window'][1]:.1f} s. "
                            f"Saturation at max {mo['sat_high_pct']:.1f}% / at idle {mo['sat_low_pct']:.1f}% of that time. "
                            f"Roll trim {mo.get('roll_trim_pwm', 0):+.0f} us, pitch trim {mo.get('pitch_trim_pwm', 0):+.0f} us, "
                            f"diagonal (yaw) imbalance {mo.get('diag_imbalance_pct', 0):+.1f}%.", BODY))

    # vibration
    el.append(Paragraph("Vibration", H2))
    f = chart_vibration(res)
    if f:
        el.append(_fig_to_img(f))
    el.append(Paragraph("Sharp peaks at tens of Hz come from props/motors (balance, bearings, loose parts). "
                        "A hump below ~5 Hz is the control loop wobbling (tuning). "
                        "Gyro noise on the ground at idle that is already high points to the airframe/mount, not the tuning.", SMALL))

    # power
    f = chart_power(res)
    if f:
        el.append(Paragraph("Power", H2))
        el.append(_fig_to_img(f))

    # tuning
    el.append(PageBreak())
    el.append(Paragraph("ADRC tuning advice", H2))
    el.append(Paragraph(
        "Your GCS exposes one ADRC value per axis (b0). In the ADRC law u = (u0 - z3)/b0 a <b>larger |b0| makes the "
        "controller softer</b> and a smaller |b0| makes it more aggressive. This tool therefore says: wobble/oscillation "
        "-> increase |b0| by 0.25 (mild) or 0.5 (strong); slow/sluggish/drifting -> decrease |b0|. "
        "For yaw (negative value) 'increase |b0|' means more negative, e.g. -1.2 -> -1.45. "
        "Confirm the direction once on your airframe: +0.25 on roll/pitch should feel lazier.", BODY))
    rows = [["Axis", "b0 this flight", "Action", "New b0", "Why", "Conf."]]
    for tn in res.tuning:
        act = {"keep": "keep", "softer": f"softer (+{tn['step']:.2f} on |b0|)",
               "stiffer": f"stiffer (-{tn['step']:.2f} on |b0|)"}[tn["action"]]
        rows.append([tn["axis"], _fmt(tn["current_b0"], 2), act, _fmt(tn["recommended_b0"], 2),
                     "; ".join(tn["reasons"]), tn["confidence"]])
    if len(rows) > 1:
        el.append(_tbl(rows, [15, 22, 33, 18, 80, 14]))
    else:
        el.append(Paragraph("No ADRC/attitude data to tune from.", BODY))
    el.append(Paragraph("Mechanical checks before touching tuning: wobble only when braking but clean hover usually means "
                        "a loose arm/motor mount, a wire touching a prop, or an autopilot mount that is too stiff or too soft - "
                        "not the gains. Fix vibration first (see Vibration section).", SMALL))

    # metadata
    el.append(Paragraph("Log contents (for traceability)", H2))
    meta_rows = [["Key", "Value"]]
    for k in ("format", "file_header", "tick_seconds", "frames", "autopilot_name", "debug_vectors", "packet_counts",
              "lingpian_zero_bias_last", "skipped_bytes", "column_mapping", "setpoint_source"):
        if k in fd.meta:
            meta_rows.append([k, str(fd.meta[k])[:600]])
    meta_rows.append(["channels", ", ".join(f"{n} ({fd.rate_of(n):.0f} Hz)" for n in sorted(fd.series)
                                            if not n.startswith("adrc_"))[:1500]])
    el.append(_tbl(meta_rows, [35, 147]))
    el.append(Paragraph("Decoding notes: RFly .rlog packet layout, time unit (100 us) and frames were reverse-engineered and "
                        "verified physically (velocity = d(position)/dt, gyro = quaternion derivative, GPS vs position). "
                        "ADRC vector elements other than [5] and [6] are not identified yet.", SMALL))

    el.append(PageBreak())
    if res.upset.get("t_start") is not None or res.upset.get("t_impact"):
        el.append(Paragraph("Post-crash procedure", H2))
        el.append(_checklist(POSTCRASH))
    el.append(Paragraph("Pre-flight checklist (before the next flight)", H2))
    el.append(_checklist(PREFLIGHT))

    def _footer(canvas, doc_):
        canvas.saveState()
        canvas.setFont("Helvetica", 7)
        canvas.setFillColor(colors.HexColor("#777777"))
        canvas.drawString(14 * mm, 7 * mm, f"dronelog_qc v{__version__} - {name}")
        canvas.drawRightString(196 * mm, 7 * mm, f"page {doc_.page}")
        canvas.restoreState()

    doc.build(el, onFirstPage=_footer, onLaterPages=_footer)
    return out_path


def _upset_story(res):
    up, fd = res.upset, res.fd
    parts = []
    t, g = res.grid_t, res.grid
    pre = []
    names = fd.motor_names()
    if names:
        mm = np.nanmean(np.vstack([g[n] for n in names]), axis=0)
        w = (t >= up["t_start"] - 2.5) & (t <= up["t_start"])
        if w.any():
            drop = np.nanmax(mm[w]) - np.nanmin(mm[w])
            if drop > 100:
                i_hi, i_lo = np.nanargmax(np.where(w, mm, -1)), np.nanargmin(np.where(w, mm, 9e9))
                pre.append(f"average motor output changed by {drop:.0f} us between t={t[i_hi]:.1f}s and t={t[i_lo]:.1f}s "
                           f"(collective/throttle change just before the upset)")
    if fd.has("x1c40_b2"):
        tt, v = fd.get("x1c40_b2")
        base = np.median(v)
        ch = tt[(v != base) & (tt > up["t_start"] - 3) & (tt < (up.get("t_impact") or up["t_upset"] + 5))]
        if ch.size:
            pre.append("status packet 0x1C/132 changed at t=" + ", ".join(f"{x:.2f}" for x in ch[:6]) +
                       "s (meaning not decoded yet - possibly a command/mode event)")
    parts.append(f"<b>1. Before:</b> normal flight from take-off ({res.summary.get('takeoff_t', 0):.1f}s), motors balanced. "
                 + ("Shortly before the upset: " + "; ".join(pre) + "." if pre else ""))
    parts.append(f"<b>2. Upset:</b> from t={up['t_start']:.2f}s the model tipped (log roll {up['roll_at_upset']:.0f}, "
                 f"pitch {up['pitch_at_upset']:.0f} deg at the 60 deg point) and reached {up['tilt_peak']:.0f} deg. "
                 f"Peak rotation {up.get('peak_rate_dps') or 0:.0f} deg/s. Motor split during the dive only "
                 f"{up.get('motor_spread_during_dive_pwm', 0):.0f} us, no motor at max.")
    if up.get("first_saturation_t"):
        parts.append(f"<b>3. Recovery attempt:</b> first motor reached max at t={up['first_saturation_t']:.2f}s "
                     "(after the model was already far over and falling).")
    if up.get("t_impact"):
        parts.append(f"<b>4. Impact:</b> t={up['t_impact']:.2f}s, {up['peak_accel_ms2'] / 9.81:.0f} g.")
    if up.get("motors_running_after_impact_s"):
        parts.append(f"<b>5. After:</b> motors kept running {up['motors_running_after_impact_s']:.0f}s after impact.")
    parts.append("<b>Interpretation:</b> a motor/prop failure normally shows one motor driven to max with the others reduced "
                 "<i>before</i> the attitude is lost. Here the controller output stayed small while the model tipped, "
                 "which points to a commanded attitude (outer loop / mode / RC input) or a control-software fault. "
                 "To prove it, the autopilot must also log the attitude setpoint and RC/mode state.")
    return "<br/>".join(parts)


# ------------------------------------------------------------------ diagnosis
SEV_RANK = {"BAD": 0, "WARN": 1, "INFO": 2, "OK": 3}


def _issues_found(res):
    """issue id -> (worst severity, [findings]) for WARN/BAD findings, ordered by severity."""
    found = {}
    for f in res.findings:
        if not f.issue or f.issue not in ISSUES or f.severity not in ("BAD", "WARN", "INFO"):
            continue
        if f.severity == "INFO" and f.issue not in ("sluggish",):
            continue
        sev, lst = found.get(f.issue, ("INFO", []))
        if SEV_RANK[f.severity] < SEV_RANK[sev]:
            sev = f.severity
        found[f.issue] = (sev, lst + [f])
    return dict(sorted(found.items(), key=lambda kv: SEV_RANK[kv[1][0]]))


def where_section(res):
    found = _issues_found(res)
    if not found:
        return [Paragraph("Where is the problem?", H2), Paragraph("No issues that need action.", BODY)]
    score = {}
    for iid, (sev, _) in found.items():
        for rank, (cause, where) in enumerate(ISSUES[iid]["causes"]):
            w = (1.0 if sev == "BAD" else 0.6) / (rank + 1)
            d = score.setdefault(where, [0.0, set()])
            d[0] += w
            d[1].add(ISSUES[iid]["title"])
    rows = [["Look here", "What this area covers", "Related issues", "Weight"]]
    for where, (w, titles) in sorted(score.items(), key=lambda kv: -kv[1][0]):
        rows.append([where, WHERE[where], "; ".join(sorted(titles)), f"{w:.1f}"])
    return [Paragraph("Where is the problem? (model / autopilot / GCS / outside)", H2),
            Paragraph("Each detected issue votes for the areas its likely causes live in (most likely cause counts most). "
                      "Start checking from the top row.", SMALL),
            _tbl(rows, [22, 55, 90, 15])]


def _checklist(items):
    rows = [["[  ]", Paragraph(x, CELL)] for x in items]
    t = Table([[Paragraph(a, CELL), b] for a, b in rows], colWidths=[10 * mm, 172 * mm])
    t.setStyle(TableStyle([("VALIGN", (0, 0), (-1, -1), "TOP"), ("TOPPADDING", (0, 0), (-1, -1), 1.5),
                           ("BOTTOMPADDING", (0, 0), (-1, -1), 1.5)]))
    return t


def issue_card(iid, sev=None, evidence=None):
    info = ISSUES[iid]
    col = SEV_COLOR.get(sev or "INFO")
    head = Table([[Paragraph(f"<font color='white'><b>{(sev + ' - ') if sev else ''}{info['title']}</b></font>", BODY)]],
                 colWidths=[182 * mm])
    head.setStyle(TableStyle([("BACKGROUND", (0, 0), (-1, -1), col), ("TOPPADDING", (0, 0), (-1, -1), 3),
                              ("BOTTOMPADDING", (0, 0), (-1, -1), 3)]))
    parts = [head]
    if evidence:
        parts.append(Paragraph("<b>What the log shows:</b> " + " | ".join(evidence), BODY))
    parts.append(Paragraph(f"<b>What it means:</b> {info['means']}", BODY))
    parts.append(Paragraph(f"<b>How it is detected:</b> {info['detect']}", SMALL))
    rows = [["#", "Likely cause (most likely first)", "Look at"]]
    for i, (c, w) in enumerate(info["causes"], 1):
        rows.append([str(i), c, w])
    parts.append(_tbl(rows, [8, 150, 24]))
    parts.append(Paragraph("<b>Check on the model (tick when done):</b>", BODY))
    parts.append(_checklist(info["checks"]))
    parts.append(Paragraph(f"<b>Fix:</b> {info['fix']}", BODY))
    parts.append(Spacer(1, 6))
    return KeepTogether(parts)


def action_plan(res):
    found = _issues_found(res)
    if not found:
        return []
    el = [PageBreak(), Paragraph("Diagnosis & action plan", H2),
          Paragraph("One card per detected issue, most serious first. Work through the checks in order - the cheapest "
                    "and most likely items come first. Re-fly and re-run this tool after each fix.", SMALL)]
    for iid, (sev, fs) in found.items():
        ev = [f"{f.title}{f' (t={f.t:.1f}s)' if f.t is not None else ''}: {f.detail}" for f in fs[:2]]
        el.append(issue_card(iid, sev, ev))
    return el
