# ADRC tuning guide (for the single b0 value in the GCS)

## What the value is
ADRC (Active Disturbance Rejection Control) estimates everything it does not model - wind,
CG offset, wrong gains - as one "total disturbance" z3 with an observer (ESO), and cancels it:

    u = (u0 - z3) / b0

`b0` is the controller's guess of **how strongly the model reacts to a motor command**
(control effectiveness ~ arm length x thrust slope / inertia). The sample log confirms the
GCS value is b0: the logged ratio element[5]/element[6] is -1.20 on yaw, the GCS yaw value.

* **Larger |b0| -> smaller output -> softer, lazier.**
* **Smaller |b0| -> bigger output -> stiffer, more aggressive, eventually oscillates.**
* Yaw is negative because the props spin the "reverse" way: the sign tells the controller which
  way the reaction torque acts. Never change the sign, only the magnitude.

## Why heavier models want a different value
Real control effectiveness = torque / inertia. More weight on the same arms/motors -> more inertia
-> less rotation per command -> the true b is *smaller*, so b0 should go *down* (more aggressive)
for the same feel. Bigger props/longer arms raise it. Batteries hung far from the centre raise
inertia a lot.

## Starting a brand-new model
1. Start from the closest known model: roll/pitch 8.0, yaw -1.3 for ~2 kg.
2. Go **softer first** for safety: roll/pitch +0.5 (e.g. 8.5), yaw -1.5.
3. Hover 30 s in position hold, then do: 3 short pitch/roll stick pulses, a 10 m/s straight leg
   with a hard stop (braking), and a 90 deg yaw turn. Log it.
4. Run the log through this tool. It reports wobble per axis.
5. Lower |b0| in 0.25 steps while the model is clean and still feels lazy (slow to stop, drifts
   after braking, slow yaw). Stop at the **first** sign of wobble, then go back +0.25.
6. Only ever change one axis per flight.

## Reading the advice
| symptom in the report | meaning | change |
|---|---|---|
| strong low-frequency wobble (> ~12 deg/s RMS, 1-6 Hz) | too aggressive | |b0| +0.5 |
| noticeable wobble (6-12 deg/s RMS) | slightly aggressive | |b0| +0.25 |
| clean, but slow to settle / drifts after stops | too soft | |b0| -0.25 |
| high gyro noise already on the ground at idle | mechanical vibration | do NOT tune - fix props/mount |
| wobble only when braking, hover clean | loose arm/mount, wire on prop, damping | mechanical first |
| motors hitting max | no thrust reserve | fix weight/props before tuning |

The direction (softer = larger |b0|) comes from the ADRC law. Confirm it once on your fleet:
fly +0.25 on roll/pitch and check the model feels lazier. If it ever feels the other way, set
`"higher_b0_means": "stiffer"` in `config/default.json` and every recommendation flips.
