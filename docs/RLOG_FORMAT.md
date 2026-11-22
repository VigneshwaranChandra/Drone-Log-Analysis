# RFly `.rlog` binary format (reverse engineered)

Source sample: `202511081618.rlog` (7.26 MB, 79,092 packets, 28 unparsed bytes = header only).
All values little-endian.

## Packet
| bytes | meaning |
|---|---|
| `41 43` | sync "AC" |
| 1 | packet id |
| 1 | length L (next packet starts L+2 bytes after the sync) |
| 2 | `pre` (varies by type; not needed) |
| 4 | tick, **unit = 100 us** |
| L-8 | payload |

File starts with `AC 00 03 "RLog" yyyymmddHHMM`.

## Packet types
| id/len | rate | content | verified by |
|---|---|---|---|
| 0x1f/64 | 200 Hz | quaternion w,x,y,z (NED world -> FRD body) + gyro p,q,r rad/s | gyro == quaternion derivative (corr 0.96-0.98, slope 1.00) |
| 0x20/80 | 200 Hz | pos N,E,D m; vel N,E,D m/s; acc N,E,D m/s^2 (earth frame, no gravity) | GPS lat/lon vs pos (corr 0.997); vel == d(pos)/dt; d(vel)/dt == acc (Z slope 1.01) |
| 0x1c/96 | 10 Hz | u8 sub-id 135, u8, u8 sats, u8 fix; 10 x f64: home lat, home lon, lat, lon, N, E, D(-MSL), vN, vE, vD | |
| 0x1c/40 | ~30 Hz | sub-id 132; bytes 2-3 normally constant, change at events; f64 ~ -baro altitude? | **not decoded** |
| 0x01/60 | 5 Hz | 4 bytes + 10 x f32 (+2 x u32); f32[0] = battery V | 6S range 24.8 -> 20.0 V |
| 0xfa/* | | named vector: 10-byte name + N x f64 | |
| ... `dianji` (电机 motor) | 20 Hz | 14 values, [0..3] = motor PWM 1000..1962, 60000 = unused | |
| ... `lingpian` (零偏 zero bias) | 20 Hz | 3 values | |
| ... `rol` `pit` `yaw` | ~370 Hz | 10 ADRC values per axis | see below |

**Time:** the 4-byte tick is 100 us. Using 1 ms makes the flight look 10x longer and every
velocity 10x too large - that mistake is easy to make, the loader auto-detects it from the
velocity/position consistency.

## ADRC vectors (`rol`/`pit`/`yaw`)
- `[6]` = controller output u. A linear fit of the 4 motor PWMs on `rol[6]`, `pit[6]`, `yaw[6]`
  explains R^2 = 0.98 of the motor differences (~9.7 us PWM per unit of u).
- `[5]/[6]` = **b0**: yaw gives -1.20 (spread -1.21..-1.19), i.e. exactly the GCS yaw value.
  Roll/pitch give ~6.2 / ~6.8 with wide spread (filtering) - approximate only.
- `[8]` ~ `[6]` (limited/filtered u). `[0..2]` look like filtered body rate. `[7]` ~ -`[5]`.
  `[3]`, `[4]`, `[9]` (always 0) not identified. No attitude **setpoint** found in the log.

## Motor order vs physics
With the user layout (M1 FR, M2 FL, M3 RL, M4 RR) the left-right and front-rear motor
differences correlate with the *other* axis of the gyro. The best physical fit is
M1 rear-left, M2 front-left, M3 front-right, M4 rear-right (M1/M3 exchanged). A board rotation
cannot explain this (it is a mirror), so either the `dianji` array order differs from the M labels
or the labels are being read from a different viewpoint. Bench test to settle it:
props off, arm, tilt the nose down by hand -> the two `dianji` values that rise are the FRONT motors.
