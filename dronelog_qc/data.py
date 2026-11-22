"""Common in-memory representation of a flight log.

Every loader (MAVLink tlog, CSV export, future RFly binary) converts its
input into a FlightData object that uses the *standard channel names* below,
so the analysis code never needs to know where the data came from.

Standard channels (all optional; analysis skips what is missing):

  Attitude      roll, pitch, yaw                [deg]   (yaw 0..360 or -180..180)
  Setpoints     roll_sp, pitch_sp, yaw_sp       [deg]
  Body rates    p, q, r                         [deg/s] (roll, pitch, yaw rate)
  Rate setpts   p_sp, q_sp, r_sp                [deg/s]
  Motors        motor_1 .. motor_N              [PWM us, or % 0..100]
  Velocity      vel_n, vel_e, vel_u             [m/s]   (u = up positive)
  Position      pos_n, pos_e, pos_u             [m]     relative to first fix/home
  GPS           lat, lon [deg], gps_sats, gps_hdop
  Altitude      alt_rel                         [m]
  Accel (body)  acc_x, acc_y, acc_z             [m/s^2]
  Vibration     vibe_x, vibe_y, vibe_z          [m/s^2]
  Power         volt [V], curr [A], batt_pct [%]
  Misc          throttle [%], groundspeed [m/s], climb [m/s]

Discrete things (mode changes, text messages, params) live in `events`,
`params` and `meta`.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np


@dataclass
class Event:
    t: float          # seconds on the log time base
    kind: str         # 'mode', 'arm', 'disarm', 'text', 'failsafe', ...
    text: str
    severity: int = 6  # MAVLink severity (0 = emergency ... 7 = debug)


@dataclass
class FlightData:
    source: str
    series: Dict[str, Tuple[np.ndarray, np.ndarray]] = field(default_factory=dict)
    events: List[Event] = field(default_factory=list)
    params: Dict[str, float] = field(default_factory=dict)
    meta: Dict[str, object] = field(default_factory=dict)

    # ------------------------------------------------------------------ basic
    def add(self, name: str, t, v) -> None:
        t = np.asarray(t, dtype=float)
        v = np.asarray(v, dtype=float)
        ok = np.isfinite(t) & np.isfinite(v)
        t, v = t[ok], v[ok]
        if t.size == 0:
            return
        order = np.argsort(t, kind="stable")
        t, v = t[order], v[order]
        # drop duplicated timestamps (keep last)
        keep = np.ones(t.size, bool)
        keep[:-1] = np.diff(t) > 0
        self.series[name] = (t[keep], v[keep])

    def has(self, *names: str) -> bool:
        return all(n in self.series and self.series[n][0].size > 1 for n in names)

    def get(self, name: str) -> Tuple[np.ndarray, np.ndarray]:
        return self.series[name]

    def motor_names(self) -> List[str]:
        names = [n for n in self.series if n.startswith("motor_")]
        return sorted(names, key=lambda s: int(s.split("_")[1]))

    def rate_of(self, name: str) -> float:
        t, _ = self.series[name]
        if t.size < 3:
            return 0.0
        return float(1.0 / np.median(np.diff(t)))

    def time_range(self) -> Tuple[float, float]:
        lo, hi = np.inf, -np.inf
        for t, _ in self.series.values():
            lo = min(lo, t[0])
            hi = max(hi, t[-1])
        return float(lo), float(hi)

    def shift_time(self, t0: float) -> None:
        """Re-base all timestamps so that t0 becomes 0."""
        for k, (t, v) in list(self.series.items()):
            self.series[k] = (t - t0, v)
        for e in self.events:
            e.t -= t0
        self.meta["time_offset_s"] = float(self.meta.get("time_offset_s", 0.0)) + t0

    # ----------------------------------------------------------- resampling
    def grid(self, rate_hz: Optional[float] = None, names: Optional[List[str]] = None):
        """Resample channels onto one uniform time grid.

        Returns (t, dict name -> values). Values outside a channel's own time
        span are NaN (no extrapolation).
        """
        names = [n for n in (names or list(self.series)) if n in self.series]
        if rate_hz is None:
            rate_hz = self.suggest_rate()
        lo, hi = self.time_range()
        t = np.arange(lo, hi, 1.0 / rate_hz)
        out = {}
        for n in names:
            ts, vs = self.series[n]
            v = np.interp(t, ts, vs)
            if n == "yaw" or n == "yaw_sp":
                # interpolate angles through the unwrapped signal
                v = np.interp(t, ts, np.degrees(np.unwrap(np.radians(vs)))) % 360.0
            v[(t < ts[0]) | (t > ts[-1])] = np.nan
            out[n] = v
        return t, out

    def suggest_rate(self) -> float:
        for key in ("roll", "p", "motor_1", "vel_n"):
            if self.has(key):
                return float(np.clip(self.rate_of(key), 2.0, 100.0))
        return 10.0
