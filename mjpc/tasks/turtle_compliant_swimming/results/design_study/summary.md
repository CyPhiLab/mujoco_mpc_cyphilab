## Attitude-locked searches

| design | speed m/s (BL/s) | vs baseline | pool-scaled m/s | stroke s | gear | elec W | elec J/m | CoT | peak / RMS motor torque N m | RMS/rated | peak motor speed rad/s | mass kg |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| `ak45_diff_bigflip_thermal` AK45-10, differential, flipper scale optimized [RMS <= rated] | 0.348 (0.53) | 2.06x | 0.43 | 0.83 | stroke 2.89:1, twist 2.84:1 | 141 | 406 | 4.19 | 6.4 / 2.5 | 1.01 | 19.3 | 9.87 |
| `ak45_diff_red_thermal` AK45-10, differential, optimized reductions [RMS <= rated] | 0.335 (0.52) | 1.99x | 0.41 | 0.70 | stroke 1.74:1, twist 1.80:1 | 131 | 392 | 4.19 | 5.1 / 2.5 | 1.01 | 19.3 | 9.54 |
| `ak45_diff_inboard_thermal` AK45-10, differential, twist motor inboard (cable) [RMS <= rated] | 0.334 (0.51) | 1.98x | 0.41 | 0.72 | stroke 1.70:1, twist 1.42:1 | 135 | 404 | 4.30 | 6.7 / 2.5 | 1.01 | 19.4 | 9.60 |
| `ak45_diff` AK45-10, yaw/roll differential, direct | 0.330 (0.51) | 1.96x | 0.41 | 0.72 | direct | 185 | 560 | 5.98 | 7.0 / 4.1 | 1.64 | 19.2 | 9.54 |
| `ak45_serial` AK45-10, serial, direct (drop-in) | 0.299 (0.46) | 1.78x | 0.37 | 0.49 | direct | 158 | 528 | 5.69 | 7.0 / 5.3 | 2.13 | 18.9 | 9.46 |
| `ak45_serial_red_thermal` AK45-10, serial, optimized reductions [RMS <= rated] | 0.293 (0.45) | 1.74x | 0.36 | 0.65 | stroke 2.32:1, twist 1.51:1 | 88 | 301 | 3.24 | 7.0 / 2.5 | 0.99 | 19.1 | 9.46 |
| `ak45_diff_coupled` AK45-10, differential + twist linkage (4 motors) | 0.245 (0.38) | 1.45x | 0.30 | 0.44 | stroke 1.12:1 | 215 | 878 | 9.86 | 7.0 / 4.3 | 1.73 | 15.9 | 9.07 |
| `xw540_3s_flipper_scale1.4` XW540-T260, 3S (11.1 V), serial, flipper x1.4 | 0.213 (0.33) | 1.27x | 0.26 | 1.49 | direct | 36 | 170 | 1.85 | 8.4 / 3.7 | n/a | 4.0 | 9.36 |
| `xw540_3s_diff_red` XW540 3S, differential with speed-up gear | 0.212 (0.33) | 1.26x | 0.26 | 0.88 | stroke 0.56:1, twist 0.59:1 | 55 | 261 | 2.93 | 8.8 / 3.4 | n/a | 4.0 | 9.07 |
| `xw540_14v8` XW540-T260 @ 14.8 V (4S), serial | 0.208 (0.32) | 1.23x | 0.26 | 1.04 | direct | 30 | 142 | 1.61 | 8.8 / 2.6 | n/a | 5.2 | 8.99 |
| `xw540_3s_red` XW540 3S, serial, optimized gear (speed-up allowed) | 0.199 (0.31) | 1.18x | 0.25 | 0.86 | stroke 0.53:1, twist 0.61:1 | 46 | 229 | 2.60 | 8.8 / 4.5 | n/a | 3.9 | 8.99 |
| `ak45_serial_thermal` AK45-10, serial, direct (drop-in) [RMS <= rated] | 0.179 (0.28) | 1.07x | 0.22 | 1.68 | direct | 36 | 198 | 2.13 | 5.3 / 2.5 | 0.99 | 10.4 | 9.46 |
| `xw540_3s_diff` XW540 3S, yaw/roll differential | 0.178 (0.27) | 1.05x | 0.22 | 1.25 | direct | 17 | 94 | 1.05 | 6.4 / 1.2 | n/a | 4.1 | 9.07 |
| `ak45_diff_passive` AK45-10, differential + passive twist (4 motors) | 0.173 (0.27) | 1.03x | 0.21 | 0.55 | stroke 1.10:1 | 208 | 1201 | 13.58 | 7.0 / 4.2 | 1.67 | 13.9 | 9.01 |
| `xw540_3s` baseline: XW540-T260, 3S (11.1 V), serial | 0.168 (0.26) | 1.00x | 0.21 | 1.31 | direct | 15 | 86 | 0.98 | 6.9 / 1.8 | n/a | 4.1 | 8.99 |

## MPC (free body)

| run | speed m/s (BL/s) | drift m/s | max tilt deg | elec W | elec J/m | mech J/m | peak / RMS motor torque N m | peak motor speed rad/s | strokes |
|---|---|---|---|---|---|---|---|---|---|
| current robot (XW540 3S) | 0.153 (0.24) | 0.021 | 24 | 22 | 145 | 98 | 7.4 / 1.8 | 4.0 | 8 |
| XW540 + speed-up differential | 0.208 (0.32) | 0.015 | 8 | 86 | 412 | 162 | 8.8 / 4.0 | 4.0 | 11 |
| AK45 serial + gears | 0.283 (0.44) | 0.071 | 20 | 102 | 362 | 232 | 7.0 / 2.8 | 19.2 | 16 |
| AK45 differential + gears | 0.328 (0.50) | 0.013 | 14 | 160 | 489 | 356 | 7.0 / 2.7 | 20.1 | 19 |
| AK45 differential + gears + flipper x1.38 | 0.337 (0.52) | 0.004 | 13 | 158 | 469 | 334 | 7.0 / 2.7 | 20.3 | 18 |
