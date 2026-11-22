"""Every detector must fire on its simulated fault and stay quiet on a healthy flight."""
import os
import sys
import tempfile
import warnings

warnings.filterwarnings("ignore")
ROOT = os.path.join(os.path.dirname(__file__), "..")
sys.path.insert(0, ROOT); sys.path.insert(0, os.path.join(ROOT, "tools"))
from simulate_rlog import simulate  # noqa: E402
from dronelog_qc.cli import load_config  # noqa: E402
from dronelog_qc.analysis.core import analyze  # noqa: E402
from dronelog_qc.loaders import load_any  # noqa: E402

EXPECT = {  # fault -> issue ids that MUST be reported as WARN/BAD
    "none": set(),
    "cg_offset": {"cg_offset"},
    "weak_motor": {"weak_motor"},
    "motor_tilt": {"yaw_imbalance"},
    "high_gain": {"wobble_fast"},
    "wind": {"wind"},
    "vibration": {"vibration"},
    "gps_glitch": {"gps_glitch"},
    "brownout": {"power_loss"},
    "motor_failure": {"motor_failure"},
    "weak_battery": {"battery_sag"},
    "overweight": {"thrust_reserve"},
    "hard_landing": {"hard_landing"},
    "esc_desync": {"esc_desync"},
}
# issues that must NOT appear for that fault (false alarms we fixed once)
FORBID = {"none": "ANY", "vibration": {"layout_mismatch", "wobble_fast"}, "motor_failure": {"layout_mismatch"},
          "high_gain": {"vibration"}, "gps_glitch": {"sluggish"}}


def run(fault):
    with tempfile.TemporaryDirectory() as d:
        p = os.path.join(d, f"{fault}.rlog")
        simulate(p, fault)
        r = analyze(load_any(p), load_config())
    return r, {f.issue for f in r.findings if f.severity in ("WARN", "BAD") and f.issue}


def test_all_faults():
    bad = []
    for fault, want in EXPECT.items():
        r, got = run(fault)
        if not want <= got:
            bad.append(f"{fault}: missing {want - got}, got {got}")
        forb = FORBID.get(fault)
        if forb == "ANY" and got:
            bad.append(f"{fault}: false alarms {got}")
        elif forb and forb != "ANY" and (forb & got):
            bad.append(f"{fault}: false alarms {forb & got}")
        if fault == "weak_motor":
            assert any("M3" in f.title for f in r.findings if f.issue == "weak_motor"), "wrong motor named"
        if fault == "motor_failure":
            assert r.upset.get("cause_motor") == 2, "wrong failed motor"
        if fault == "wind":
            assert 6.5 < r.wind["speed_ms"] < 9.5 and 250 < r.wind["from_deg"] < 290, r.wind
        if fault == "high_gain":
            assert any(t["axis"] == "Roll" and t["action"] == "softer" for t in r.tuning)
    assert not bad, "\n".join(bad)


if __name__ == "__main__":
    test_all_faults(); print("all detectors ok")
