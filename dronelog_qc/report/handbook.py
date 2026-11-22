"""Builds the reference handbook PDF (everything a QC engineer needs next to the tool)."""
from __future__ import annotations

import datetime as _dt

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, FancyArrowPatch, FancyBboxPatch
from reportlab.lib.pagesizes import A4
from reportlab.lib.units import mm
from reportlab.platypus import PageBreak, Paragraph, SimpleDocTemplate, Spacer

from .. import __version__
from ..knowledge import ISSUES, POSTCRASH, PREFLIGHT, WHERE
from .pdf import BODY, H1, H2, SMALL, _checklist, _fig_to_img, _tbl, issue_card

H3 = H2.clone("h3", fontSize=10.5, spaceBefore=6, spaceAfter=2)


def P(t):
    return Paragraph(t, BODY)


def S(t):
    return Paragraph(t, SMALL)


# ------------------------------------------------------------------ figures
def fig_layout(cfg_layout):
    pos = {"front-right": (1, 1), "front-left": (-1, 1), "rear-left": (-1, -1), "rear-right": (1, -1)}
    fig, ax = plt.subplots(figsize=(4.4, 4.4))
    ax.plot([-1, 1], [-1, 1], color="#555", lw=6, solid_capstyle="round")
    ax.plot([-1, 1], [1, -1], color="#555", lw=6, solid_capstyle="round")
    ax.add_patch(FancyBboxPatch((-0.28, -0.35), 0.56, 0.7, boxstyle="round,pad=0.02", color="#263238"))
    ax.annotate("", xy=(0, 0.3), xytext=(0, -0.2), arrowprops=dict(arrowstyle="-|>", color="#ffd54f", lw=2.5))
    for m, p in cfg_layout.items():
        if p not in pos:
            continue
        x, y = pos[p]
        ax.add_patch(Circle((x, y), 0.42, color="#90caf9", alpha=0.55))
        ax.text(x, y, m.replace("motor_", "M"), ha="center", va="center", fontsize=14, weight="bold")
        ax.text(x, y - 0.6, p, ha="center", fontsize=8)
    ax.text(0, 1.55, "FRONT (nose)", ha="center", fontsize=10, weight="bold")
    ax.set_xlim(-1.7, 1.7); ax.set_ylim(-1.8, 1.8); ax.set_aspect("equal"); ax.axis("off")
    ax.set_title("Configured layout (top view)", fontsize=10)
    return fig


def fig_adrc():
    fig, ax = plt.subplots(figsize=(9, 2.8))
    ax.axis("off"); ax.set_xlim(0, 10); ax.set_ylim(0, 3)
    def box(x, y, w, t, c="#e3f2fd"):
        ax.add_patch(FancyBboxPatch((x, y), w, 0.8, boxstyle="round,pad=0.05", fc=c, ec="#1565c0"))
        ax.text(x + w / 2, y + 0.4, t, ha="center", va="center", fontsize=8.5)
    def arr(a, b):
        ax.add_patch(FancyArrowPatch(a, b, arrowstyle="-|>", mutation_scale=12, color="#333"))
    box(0.1, 1.9, 1.6, "attitude\nsetpoint")
    box(2.2, 1.9, 1.8, "u0 = PD on error\n(controller bandwidth)")
    box(4.6, 1.9, 2.0, "u = (u0 - z3) / b0", "#fff3e0")
    box(7.2, 1.9, 1.3, "motors\n(mixer)")
    box(8.8, 1.9, 1.1, "model")
    box(4.6, 0.3, 2.0, "ESO observer:\nz1 angle, z2 rate,\nz3 = total disturbance", "#e8f5e9")
    arr((1.7, 2.3), (2.2, 2.3)); arr((4.0, 2.3), (4.6, 2.3)); arr((6.6, 2.3), (7.2, 2.3)); arr((8.5, 2.3), (8.8, 2.3))
    arr((9.35, 1.9), (6.6, 0.7)); arr((5.6, 1.1), (5.6, 1.9)); arr((4.6, 0.7), (3.1, 1.9))
    ax.text(5.6, 2.95, "b0 = the ONE value in your GCS: larger |b0| -> smaller u -> softer", ha="center", fontsize=9,
            color="#bf360c", weight="bold")
    return fig


# ------------------------------------------------------------------ document
def build_handbook(out_path, cfg):
    doc = SimpleDocTemplate(out_path, pagesize=A4, leftMargin=14 * mm, rightMargin=14 * mm,
                            topMargin=12 * mm, bottomMargin=12 * mm, title="Drone Log QC Handbook")
    el = []
    el.append(Paragraph("Drone Log QC - Handbook", H1))
    el.append(S(f"Version {__version__} - {_dt.date.today():%Y-%m-%d}. For the RFly autopilot (.rlog, ADRC control) "
                "and MAVLink/CSV logs. Plain-language reference to use next to the QC report."))
    toc = ["1. What the tool does and how to use it", "2. How to read the QC report", "3. Your log: what each channel means",
           "4. Physics you need: frames, motors, what motor patterns mean", "5. ADRC tuning (the b0 value)",
           "6. Crash investigation procedure", "7. Where is the problem? Quick symptom guide",
           "8. Issue library (every detectable problem)", "9. Checklists", "10. The sample crash log",
           "11. Where the autopilot software comes from", "12. Limits, and what to ask the vendor",
           "Appendix: .rlog binary format"]
    el.append(_tbl([["Contents"]] + [[x] for x in toc], [182]))

    # 1 ---------------------------------------------------------------
    el.append(PageBreak()); el.append(Paragraph("1. What the tool does and how to use it", H2))
    el.append(P("Give it a flight log, get a <b>PDF QC report</b> and an <b>offline 3D replay</b>. No internet, no data "
                "leaves the computer. Supported: RFly <b>.rlog</b> (from the autopilot SD card), MAVLink <b>.tlog</b> "
                "(GCS telemetry), and <b>CSV</b> exports from any viewer (column names mapped in config/channel_map.json)."))
    el.append(_tbl([["Way to run", "How"],
                    ["Windows app", "Run build_windows_exe.bat once -> dist\\DroneLogQC.exe. Double-click, pick one or more logs. "
                                    "The PDF and the 3D replay open automatically; files are saved in a 'qc_reports' folder next to the log."],
                    ["Command line", "python -m dronelog_qc flight.rlog --rp-b0 8.0 --yaw-b0 -1.4 --mass 2.0"],
                    ["Many logs", "python -m dronelog_qc *.rlog -o reports"],
                    ["Settings", "config/default.json: motor layout, PWM range, thresholds, tuning step sizes, mass."]],
                   [30, 152]))
    el.append(P("Always enter the b0 values that were set for that flight (window fields or --rp-b0 / --yaw-b0). The tool "
                "can read yaw b0 back from the log exactly, roll/pitch only approximately."))

    # 2 ---------------------------------------------------------------
    el.append(Paragraph("2. How to read the QC report", H2))
    el.append(_tbl([["Part", "What it tells you"],
                    ["QC verdict", "FAIL = at least one BAD finding (do not fly until fixed). PASS WITH WARNINGS = WARN items. PASS = clean."],
                    ["Levels", "BAD = safety or crash relevant. WARN = needs attention. INFO = context. OK = checked and fine."],
                    ["Confidence", "high = measured directly. medium = interpretation backed by physics. low = a hint, confirm on the model."],
                    ["Where is the problem?", "Ranks the areas (propulsion, GCS parameters, firmware, autopilot sensors, power, outside, airframe) "
                                              "by how many detected issues point at them. Start checking at the top."],
                    ["Diagnosis & action plan", "One card per issue: evidence, meaning, causes ranked, a tick-box check list, the fix."],
                    ["Crash reconstruction", "Step-by-step story with the zoomed plot: before, upset, recovery attempt, impact, after."],
                    ["Movement timeline", "What the model was doing: ground, climb, hover, cruise, turn, braking, descent, UPSET."],
                    ["Motors / Vibration / Power", "Balance, layout physics check, saturation, gyro spectrum, voltage and sag."],
                    ["ADRC tuning advice", "Per axis: keep / softer / stiffer and the new b0 value (+/-0.25 or +/-0.5)."],
                    ["3D replay (HTML)", "Drag to rotate, wheel to zoom, space to play. Motor dots go green -> red with output. Red track = upset."]],
                   [38, 144]))
    el.append(P("All times are seconds from the first record in the log. The autopilot starts logging some time after power-on "
                "(the report shows the autopilot uptime at log start), so log time 0 is not power-on."))

    # 3 ---------------------------------------------------------------
    el.append(PageBreak()); el.append(Paragraph("3. Your log: what each channel means", H2))
    el.append(P("The RflySim log replayer shows the .rlog with Chinese-pinyin names. What they really are (verified on a real log):"))
    el.append(_tbl([["Viewer name", "What it is", "Units / notes"],
                    ["AttitudeAngle / AttitudeQuat", "Attitude: roll, pitch, yaw (quaternion inside the file)", "deg. File stores quaternion NED -> body FRD, 200 Hz"],
                    ["AngularRate", "Gyro body rates p (roll), q (pitch), r (yaw)", "rad/s in file, 200 Hz. Spikes = sharp kicks / impacts"],
                    ["MSPositionENU", "Position from home. File stores North, East, Down", "m. Viewer label says ENU: x=East, y=North, z=Up"],
                    ["MSVelocityENU", "Velocity", "m/s. 0 on an axis = not moving along it"],
                    ["MSAccelerationENU", "Kinematic acceleration (gravity removed)", "m/s^2. Impacts show as huge spikes"],
                    ["DVec14 dianji (= motor)", "Motor COMMANDS [0..3] = M1..M4", "PWM us: 1000 stop, 1250 idle, 1962 max. Not RPM. 60000 = unused slot. 20 Hz"],
                    ["DVec lingpian (= zero bias)", "Zero bias / offsets (3 values)", "Sensor bias estimates"],
                    ["DVec rol / pit / yaw", "ADRC controller internals per axis (10 values)", "[6] = controller output u; [5] = b0*u; others not fully decoded. NOT stick inputs"],
                    ["PosSensor (0x1c/135)", "GPS: home lat/lon, lat/lon, N/E/D, velocity, satellites, fix", "10 Hz"],
                    ["0x1c/132", "Status packet; changes at events (possibly commands/mode)", "Not decoded yet"],
                    ["CurrentSensor (0x01)", "Battery: first value = voltage", "V, 5 Hz. Other values not decoded"]],
                   [36, 72, 74]))
    el.append(S("Corrections to earlier notes: 'dianji' are commanded PWM, not RPM. 'DVec rol/pit/yaw' are controller internals, not "
                "commanded angles - this log has NO attitude setpoint channel, which is why some crash causes cannot be proven. "
                "The tick in the file is 100 microseconds; reading it as 1 ms makes the flight 10x longer and speeds 10x too high."))
    el.append(Paragraph("Reading a graph", H3))
    el.append(P("Bottom axis = time, left axis = value. Flat plateaus in velocity = steady straight legs; the same legs are "
                "straight sloped lines in position; corners of the route are the peaks/valleys in position. Vertical velocity near 0 "
                "= altitude hold. Ground speed on a diagonal leg = sqrt(Vx^2 + Vy^2) (9 and -4 m/s -> 9.85 m/s)."))

    # 4 ---------------------------------------------------------------
    el.append(PageBreak()); el.append(Paragraph("4. Physics you need", H2))
    el.append(_fig_to_img(fig_layout(cfg["airframe"]["motor_layout"]), 85))
    el.append(P("<b>Frames.</b> Body: x forward (nose), y right, z down. Roll + = right side down, pitch + = nose up, "
                "yaw + = nose turns right (clockwise from above). World: North, East, Down (viewer shows East, North, Up)."))
    el.append(P("<b>How a quad moves.</b> All four motors up = more lift. To roll right, the LEFT motors speed up. To pitch "
                "nose-up, the FRONT motors speed up. To yaw, one diagonal pair speeds up (the props that spin opposite to the "
                "wanted turn). To move forward, it first pitches nose-down, so it briefly speeds the REAR motors."))
    el.append(Paragraph("What motor patterns mean", H3))
    el.append(_tbl([["Pattern in the motor graph", "Short-lived (manoeuvre)", "Constant for the whole flight"],
                    ["All four together up/down", "climb / descent, braking load, gust", "heavy model or sagging battery (high average)"],
                    ["Front pair higher", "pitching nose-up: braking while flying forward", "CG forward (nose heavy)"],
                    ["Rear pair higher", "pitching nose-down: accelerating forward", "CG rear; in fast forward flight rear props work in "
                                                                                       "front props' downwash - mild rear bias is normal"],
                    ["Left pair higher", "rolling right: moving right / stopping a leftward move", "CG left"],
                    ["One diagonal pair higher", "yawing", "twisted arm / tilted motor / mismatched prop"],
                    ["One motor alone higher", "-", "that motor/prop/ESC is weak (less thrust per command)"],
                    ["One motor at max, opposite at min", "thrust loss on the maxed motor (prop/motor/ESC) - emergency", "-"],
                    ["All motors close together while the model tips over", "the controller did not try to stop it (commanded / software)", "-"]],
                   [52, 65, 65]))
    el.append(P("<b>Wind.</b> A multirotor leans INTO the wind to hold position. Leaning needs no extra motor difference once the "
                "lean is reached, so wind shows mostly as tilt + higher average output, and as different speeds/tilts on legs "
                "flown in different directions. The tool fits wind from the legs (direction and speed)."))
    el.append(P("<b>Vibration vs. wobble.</b> Vibration = tens of Hz (prop/motor rotation) - a mechanical problem. "
                "Wobble = 1-10 Hz rocking - control loop (tuning) or loose structure. Slow sway < 1.5 Hz with overshoot - controller too soft."))

    # 5 ---------------------------------------------------------------
    el.append(PageBreak()); el.append(Paragraph("5. ADRC tuning - the b0 value", H2))
    el.append(_fig_to_img(fig_adrc(), 175))
    el.append(P("ADRC (Active Disturbance Rejection Control) does not need an exact model. An observer (ESO) estimates "
                "everything unknown - wind, CG offset, imperfect motors - as one 'total disturbance' z3, and the control law "
                "cancels it: <b>u = (u0 - z3) / b0</b>. b0 is the controller's estimate of how strongly the model responds to a "
                "command. On the real log, element[5]/element[6] = -1.20 on yaw, which is exactly the GCS yaw value, so the "
                "value in your GCS is b0."))
    el.append(_tbl([["Change", "Effect", "When"],
                    ["|b0| up (e.g. 8.0 -> 8.25, yaw -1.2 -> -1.45)", "softer, smoother, slower; less wobble", "fast wobble, shaking, motor noise"],
                    ["|b0| down (8.0 -> 7.75, yaw -1.2 -> -0.95)", "stiffer, sharper; eventually wobbles", "slow, swaying, overshoots after stops"],
                    ["sign", "never change - yaw is negative because of the reversed props", "-"]],
                   [62, 62, 58]))
    el.append(Paragraph("Why weight changes the value", H3))
    el.append(P("Real control effectiveness = arm x thrust slope / inertia. Heavier model or weight far from the centre -> more "
                "inertia -> the true b is smaller -> lower b0 for the same feel. Bigger props / longer arms / more powerful motors "
                "-> higher. Your base values (roll/pitch 7.5-8.4, yaw -1.2 to -1.4 for ~2 kg) are consistent with this."))
    el.append(Paragraph("Starting a brand-new model", H3))
    el.append(_checklist([
        "Copy b0 from the closest known model (same motors/props/arm length, similar weight).",
        "Start SOFTER for safety: roll/pitch +0.5, yaw 0.2 more negative.",
        "Fly the test pattern: 30 s hover; 3 short roll and pitch stick pulses; a 10 m/s straight leg with a hard stop; "
        "the same leg back; a 90 deg yaw turn; land. Log it.",
        "Run the tool. Fix every mechanical / vibration / propulsion finding first - tuning cannot fix them.",
        "If clean and lazy: lower |b0| by 0.25 on ONE axis, fly the same pattern, compare reports.",
        "Stop at the first sign of fast wobble, then go back 0.25. That is your value.",
        "Do yaw last. Record each step (date, model, weight, battery, b0 values, report verdict).",
    ]))
    el.append(_tbl([["Report says", "Step"], ["strong fast wobble (> 2x limit)", "|b0| +0.5"], ["fast wobble", "|b0| +0.25"],
                    ["clean", "keep"], ["slow to stop, overshoot, no wobble", "|b0| -0.25 (low confidence, confirm by feel)"],
                    ["wobble only when braking", "mechanical checks first; then |b0| +0.25"],
                    ["vibration / weak motor / saturation", "no tuning change until fixed"]], [90, 92]))
    el.append(S("If one test flight shows +0.25 makes the model feel MORE aggressive on your autopilot, set "
                "\"higher_b0_means\": \"stiffer\" in config/default.json - every recommendation flips automatically."))

    # 6 ---------------------------------------------------------------
    el.append(PageBreak()); el.append(Paragraph("6. Crash investigation procedure", H2))
    el.append(_checklist([
        "Secure: disconnect battery. Photograph everything before touching. Copy the .rlog and GCS logs.",
        "Run the tool. Read the crash reconstruction page first.",
        "Find the impact (acceleration spike) and the upset start (tilt leaves normal). Work backwards 3-5 s from the upset start.",
        "Ask: did the controller fight? Look at motors in the 1-2 s before and during the tip-over.",
        "Classify with the table below, then follow the matching issue card.",
        "Confirm physically (props, motors, ESCs, connectors, mount) and with the pilot (stick, switches, GCS actions).",
        "Write it down: time, cause, evidence, fix. Re-fly with a hover test before the next mission.",
    ]))
    el.append(_tbl([["Type", "Signature in the log", "Most likely"],
                    ["Motor / prop failure", "one motor jumps to MAX (asks for thrust it cannot get), opposite motor drops; model rolls to "
                                             "that corner; sharp jagged gyro spike", "prop loss, ESC/motor failure, connector"],
                    ["Uncommanded-looking upset", "model tips over while motors stay close together and none saturates", "commanded attitude "
                     "(mode/RC/failsafe), control-software fault, attitude estimate error"],
                    ["Power loss / reset", "log ends in the air; or all motors drop together; voltage collapse just before",
                     "battery/connector/BEC, firmware reset"],
                    ["GPS glitch / fly-away", "position jumps without matching velocity; controller then flies hard to 'correct'",
                     "multipath, interference, GPS hardware"],
                    ["Collision", "impact spike while tilt and motors were normal", "obstacle, terrain, altitude error"],
                    ["Saturation crash", "motors at max for seconds before the upset (heavy, wind, aggressive manoeuvre)",
                     "overweight, wind, speed/brake limits"],
                    ["Hard landing", "vertical speed > 2-3 m/s at touchdown", "landing speed setting, pilot, battery sag"]],
                   [32, 95, 55]))
    el.append(S("AngularRate check: a sudden jagged spike = hardware event (prop/motor/impact); a smooth rise = commanded or "
                "control-driven motion."))

    # 7 ---------------------------------------------------------------
    el.append(PageBreak()); el.append(Paragraph("7. Where is the problem? Quick symptom guide", H2))
    el.append(_tbl([["Area", "What it covers"]] + [[k, v] for k, v in WHERE.items()], [26, 156]))
    el.append(Spacer(1, 4))
    el.append(_tbl([["Symptom", "Check first", "Then", "Not usually"],
                    ["Wobble in hover", "props, arms, autopilot mount", "b0 +0.25", "battery"],
                    ["Wobble only when braking", "arm/gear play, wires near props, mount", "b0 +0.25, brake limits", "GPS"],
                    ["Drifts / circles in hover", "compass calibration, GPS placement", "wind, position params", "b0"],
                    ["One motor always higher", "prop, motor bearing, ESC calibration", "arm straightness", "tuning"],
                    ["Two neighbour motors higher", "battery/payload position", "wind that day", "tuning"],
                    ["Diagonal pair higher", "motor tilt, twisted arm, prop model", "weak motor", "b0 yaw"],
                    ["Motors hit max", "weight, props, battery", "speed/climb/brake limits", "b0"],
                    ["Voltage sags hard", "battery IR, connector", "current draw / weight", "-"],
                    ["Position jumps", "GPS antenna placement, interference", "sky view", "tuning"],
                    ["Altitude bobbing", "baro foam, vibration", "altitude loop", "b0"],
                    ["Model tips over, motors equal", "pilot/mode/failsafe events", "vendor: setpoint logging, firmware", "props"],
                    ["Log ends in air", "battery connector, BEC", "firmware reset", "-"]],
                   [44, 52, 48, 38]))

    # 8 ---------------------------------------------------------------
    el.append(PageBreak()); el.append(Paragraph("8. Issue library", H2))
    el.append(S("Every problem the tool can detect. The QC report shows only the cards for issues found in that log."))
    for iid in ISSUES:
        el.append(issue_card(iid))

    # 9 ---------------------------------------------------------------
    el.append(PageBreak()); el.append(Paragraph("9. Checklists", H2))
    el.append(Paragraph("Pre-flight", H3)); el.append(_checklist(PREFLIGHT))
    el.append(Paragraph("Post-crash", H3)); el.append(_checklist(POSTCRASH))
    el.append(Paragraph("Tuning flight record (copy per flight)", H3))
    el.append(_tbl([["Date", "Model / weight", "Battery", "Roll/Pitch b0", "Yaw b0", "Wind", "Verdict", "Notes"],
                    ["", "", "", "", "", "", "", ""], ["", "", "", "", "", "", "", ""], ["", "", "", "", "", "", "", ""]],
                   [18, 30, 18, 22, 18, 16, 20, 40]))

    # 10 --------------------------------------------------------------
    el.append(PageBreak()); el.append(Paragraph("10. The sample crash log (202511081618.rlog)", H2))
    el.append(_tbl([["t [s]", "Event"],
                    ["0.9", "motors start, idle 1250 us"], ["5.9", "take-off; climbs to 35 m drifting south-west sideways"],
                    ["5.9-16", "normal flight; motors balanced within ~2%; average output 56%"],
                    ["15.1-15.7", "average motor output drops ~200 us; status packet 0x1c/132 changes at 15.62 s"],
                    ["16.2", "tips over: < 8 -> 60 deg in 0.46 s, peak 83 deg, 165 deg/s. Motor split only 70 us, none at max"],
                    ["16.2-17", "acceleration follows the tilt (corr 0.82): the tilt was real, not an estimator glitch"],
                    ["18.9", "first motor at max - only after it was already falling"],
                    ["23.17", "ground impact 28 g"], ["23-50", "motors keep running 26 s, model upside down"]], [22, 160]))
    el.append(P("<b>Conclusion (medium confidence):</b> not a classic motor/prop failure - the controller did not fight the "
                "tip-over. Most likely a commanded attitude (mode/RC/failsafe/outer loop) or a control-software fault. Cannot be "
                "proven without the attitude setpoint and RC/mode channels in the log. Separate issues: high gyro vibration already "
                "on the ground, no crash auto-disarm, strong battery sag, and the motor numbering in the log does not match the "
                "stated layout (M1/M3 appear exchanged - do the bench test)."))

    # 11 --------------------------------------------------------------
    el.append(Paragraph("11. Where the autopilot software comes from", H2))
    el.append(P("<b>Facts in the log:</b> the viewer is the RflySim Log Replayer (Reliable Flight Control group, Beihang "
                "University, China). Field names written by the firmware developers are Chinese pinyin (dianji = motor, "
                "lingpian = zero bias). Control is ADRC with an extended state observer, a method from Chinese control research "
                "(Han Jingqing) that is rare in ArduPilot/PX4. The format is neither ArduPilot .bin nor PX4 .ulg."))
    el.append(P("<b>What that does NOT prove:</b> where the board is designed or built, or who owns the IP. An Indian company can "
                "license, adapt or co-develop such software. 'RFLYSENSE / RFLY Innovations' could not be confirmed by web search."))
    el.append(_checklist([
        "MCA portal (mca.gov.in): company CIN, directors, incorporation date, address.",
        "DGCA / QCI type certificate and Digital Sky listing: which flight controller, which manufacturer.",
        "Open the autopilot: chip part numbers, PCB silkscreen and date codes.",
        "GCS / viewer .exe -> Properties -> Details: company and copyright; install folder: Chinese docs or DLLs.",
        "USB VID/PID of the autopilot in Device Manager -> look up the vendor.",
        "Ask in writing: who owns the flight-control source code; ask for the log format spec and ADRC parameter documentation.",
    ]))
    el.append(S("Indian government/defence tenders restrict Chinese components and software in drones; if 'indigenous' is claimed "
                "in such tenders, the software origin matters - raise it through proper channels, with evidence."))

    # 12 --------------------------------------------------------------
    el.append(Paragraph("12. Limits, and what to ask the vendor", H2))
    el.append(P("The tool only knows what is logged. To make every crash provable, ask the vendor to add to the .rlog:"))
    el.append(_checklist(["Attitude and rate SETPOINTS (roll/pitch/yaw)", "RC stick inputs and flight mode / mission item",
                          "Failsafe and arming events (text messages)", "Battery current (A) and consumed mAh",
                          "ESC telemetry: RPM, temperature, current per motor", "Raw IMU (accel/gyro) at >= 400 Hz for vibration",
                          "Crash-detect auto-disarm", "Documentation of all DVec elements and the log format"]))
    el.append(P("Telemetry logs (.tlog) are usually 2-10 Hz: fine for flight path, battery and events, too slow for wobble and "
                "vibration analysis. Use the SD-card .rlog for those."))

    # appendix --------------------------------------------------------
    el.append(PageBreak()); el.append(Paragraph("Appendix: .rlog binary format (reverse engineered and verified)", H2))
    el.append(_tbl([["Item", "Layout"],
                    ["Packet", "'A' 'C' | id u8 | length L u8 | pre u16 | tick u32 (100 us) | payload. Next packet at +L+2"],
                    ["0x00", "header 'RLog' + yyyymmddHHMM"],
                    ["0x1f / 64", "7 x f64: quaternion w,x,y,z; gyro p,q,r rad/s (200 Hz)"],
                    ["0x20 / 80", "9 x f64: pos N,E,D; vel N,E,D; acc N,E,D (200 Hz)"],
                    ["0x1c / 96", "u8 135, u8, u8 sats, u8 fix, 4 bytes; 10 x f64: home lat, lon, lat, lon, N, E, D, vN, vE, vD"],
                    ["0x1c / 40", "u8 132 ... status (not decoded)"],
                    ["0x01 / 60", "4 bytes, 10 x f32 (f32[0] = battery V), 2 x u32"],
                    ["0xfa / *", "10-byte name + N x f64 (dianji 14, lingpian 3, rol/pit/yaw 10)"]], [24, 158]))
    el.append(S("Verification: velocity == d(position)/dt only with the 100 us tick; gyro == quaternion derivative (corr 0.96-0.98, "
                "slope 1.00); GPS lat/lon vs position corr 0.997; motor PWM differences explained by ADRC outputs with R^2 = 0.98."))

    doc.build(el)
    return out_path
