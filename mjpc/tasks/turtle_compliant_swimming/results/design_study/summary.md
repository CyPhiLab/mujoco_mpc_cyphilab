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

## Speed-efficiency Pareto fronts (attitude locked)

Electrical energy per metre of the motors, minimized at each speed; 'total' adds 10 W of electronics.

| design | speed m/s | motor J/m | motor W | total J/m | stroke s | knobs |
|---|---|---|---|---|---|---|
| current robot: XW540 3S | 0.059 | 17 | 1.0 | 185 | 3.19 |  |
| current robot: XW540 3S | 0.089 | 33 | 3.0 | 145 | 3.37 |  |
| current robot: XW540 3S **(range-optimal)** | 0.120 | 47 | 5.6 | 130 | 2.48 |  |
| current robot: XW540 3S | 0.149 | 67 | 10.0 | 134 | 1.49 |  |
| current robot: XW540 3S | 0.168 | 86 | 14.5 | 146 | 1.31 |  |
| XW540 3S, flipper size free | 0.063 | 21 | 1.3 | 181 | 3.82 | flipper_scale 0.95 |
| XW540 3S, flipper size free | 0.089 | 33 | 3.0 | 145 | 2.57 | flipper_scale 0.89 |
| XW540 3S, flipper size free | 0.119 | 50 | 6.0 | 134 | 2.10 | flipper_scale 0.92 |
| XW540 3S, flipper size free **(range-optimal)** | 0.149 | 67 | 9.9 | 134 | 1.57 | flipper_scale 0.96 |
| XW540 3S, flipper size free | 0.182 | 106 | 19.3 | 161 | 1.38 | flipper_scale 1.10 |
| XW540 4S (14.8 V) | 0.060 | 17 | 1.0 | 184 | 3.82 |  |
| XW540 4S (14.8 V) | 0.092 | 42 | 3.8 | 151 | 2.20 |  |
| XW540 4S (14.8 V) **(range-optimal)** | 0.120 | 59 | 7.0 | 142 | 2.47 |  |
| XW540 4S (14.8 V) | 0.152 | 82 | 12.5 | 148 | 1.92 |  |
| XW540 4S (14.8 V) | 0.179 | 108 | 19.3 | 164 | 1.29 |  |
| XW540 4S (14.8 V) | 0.208 | 142 | 29.6 | 191 | 1.04 |  |
| XW540 3S + differential + speed-up | 0.062 | 20 | 1.3 | 181 | 3.55 | g_roll 1.50, g_twist 0.83 |
| XW540 3S + differential + speed-up | 0.090 | 30 | 2.7 | 141 | 3.25 | g_roll 1.48, g_twist 1.08 |
| XW540 3S + differential + speed-up **(range-optimal)** | 0.119 | 43 | 5.1 | 127 | 2.11 | g_roll 1.50, g_twist 0.89 |
| XW540 3S + differential + speed-up | 0.152 | 71 | 10.9 | 137 | 1.42 | g_roll 1.19, g_twist 1.31 |
| XW540 3S + differential + speed-up | 0.182 | 132 | 24.1 | 187 | 0.88 | g_roll 0.89, g_twist 0.86 |
| XW540 3S + differential + speed-up | 0.212 | 261 | 55.3 | 308 | 0.88 | g_roll 0.56, g_twist 0.59 |
| AK45-10 serial + gears | 0.064 | 19 | 1.2 | 176 | 3.81 | g_roll 2.44, g_twist 1.01 |
| AK45-10 serial + gears **(range-optimal)** | 0.091 | 37 | 3.4 | 147 | 2.39 | g_roll 2.44, g_twist 0.93 |
| AK45-10 serial + gears | 0.120 | 70 | 8.3 | 153 | 2.57 | g_roll 1.67, g_twist 0.51 |
| AK45-10 serial + gears | 0.150 | 96 | 14.4 | 163 | 1.53 | g_roll 2.69, g_twist 1.20 |
| AK45-10 serial + gears | 0.185 | 138 | 25.7 | 192 | 1.55 | g_roll 2.54, g_twist 0.86 |
| AK45-10 serial + gears | 0.214 | 146 | 31.2 | 193 | 1.12 | g_roll 3.39, g_twist 0.88 |
| AK45-10 serial + gears | 0.242 | 199 | 48.3 | 241 | 0.95 | g_roll 2.62, g_twist 0.86 |
| AK45-10 serial + gears | 0.270 | 228 | 61.6 | 265 | 0.98 | g_roll 2.34, g_twist 2.59 |
| AK45-10 serial + gears | 0.293 | 301 | 88.1 | 335 | 0.65 | g_roll 2.32, g_twist 1.51 |
| AK45-10 differential + gears | 0.063 | 21 | 1.3 | 181 | 3.76 | g_roll 1.79, g_twist 0.76 |
| AK45-10 differential + gears **(range-optimal)** | 0.090 | 34 | 3.1 | 146 | 2.85 | g_roll 2.51, g_twist 1.17 |
| AK45-10 differential + gears | 0.122 | 72 | 8.8 | 154 | 1.71 | g_roll 1.49, g_twist 0.53 |
| AK45-10 differential + gears | 0.149 | 106 | 15.8 | 173 | 1.83 | g_roll 1.59, g_twist 0.67 |
| AK45-10 differential + gears | 0.182 | 130 | 23.7 | 185 | 1.07 | g_roll 2.84, g_twist 1.44 |
| AK45-10 differential + gears | 0.208 | 180 | 37.4 | 228 | 1.28 | g_roll 1.93, g_twist 1.22 |
| AK45-10 differential + gears | 0.238 | 208 | 49.5 | 250 | 1.21 | g_roll 2.25, g_twist 1.14 |
| AK45-10 differential + gears | 0.274 | 238 | 65.3 | 274 | 0.96 | g_roll 2.73, g_twist 0.82 |
| AK45-10 differential + gears | 0.299 | 275 | 82.2 | 309 | 0.69 | g_roll 2.50, g_twist 2.09 |
| AK45-10 differential + gears | 0.335 | 392 | 131.3 | 421 | 0.70 | g_roll 1.74, g_twist 1.80 |
| AK45-10 differential + gears, flipper free | 0.075 | 28 | 2.1 | 160 | 3.79 | g_roll 1.67, g_twist 1.13, flipper_scale 1.11 |
| AK45-10 differential + gears, flipper free | 0.090 | 39 | 3.5 | 150 | 3.20 | g_roll 2.15, g_twist 1.96, flipper_scale 1.00 |
| AK45-10 differential + gears, flipper free **(range-optimal)** | 0.120 | 55 | 6.6 | 138 | 3.63 | g_roll 2.58, g_twist 0.54, flipper_scale 1.24 |
| AK45-10 differential + gears, flipper free | 0.159 | 107 | 17.1 | 170 | 2.32 | g_roll 2.00, g_twist 1.45, flipper_scale 1.09 |
| AK45-10 differential + gears, flipper free | 0.181 | 152 | 27.6 | 207 | 1.21 | g_roll 1.56, g_twist 1.26, flipper_scale 0.97 |
| AK45-10 differential + gears, flipper free | 0.209 | 190 | 39.7 | 238 | 1.73 | g_roll 1.77, g_twist 0.66, flipper_scale 1.04 |
| AK45-10 differential + gears, flipper free | 0.241 | 202 | 48.6 | 243 | 1.52 | g_roll 2.70, g_twist 1.20, flipper_scale 1.14 |
| AK45-10 differential + gears, flipper free | 0.269 | 212 | 57.0 | 249 | 1.42 | g_roll 3.00, g_twist 0.95, flipper_scale 1.14 |
| AK45-10 differential + gears, flipper free | 0.300 | 297 | 88.9 | 330 | 0.53 | g_roll 2.84, g_twist 2.01, flipper_scale 1.07 |
| AK45-10 differential + gears, flipper free | 0.329 | 335 | 110.5 | 366 | 0.98 | g_roll 2.47, g_twist 1.25, flipper_scale 1.18 |
| AK45-10 differential + gears, flipper free | 0.348 | 406 | 141.0 | 434 | 0.83 | g_roll 2.89, g_twist 2.84, flipper_scale 1.38 |

## MPC (free body)

| run | speed m/s (BL/s) | drift m/s | max tilt deg | elec W | elec J/m | mech J/m | peak / RMS motor torque N m | peak motor speed rad/s | strokes |
|---|---|---|---|---|---|---|---|---|---|
| current robot (XW540 3S) | 0.153 (0.24) | 0.021 | 24 | 22 | 145 | 98 | 7.4 / 1.8 | 4.0 | 8 |
| XW540 + speed-up differential | 0.208 (0.32) | 0.015 | 8 | 86 | 412 | 162 | 8.8 / 4.0 | 4.0 | 11 |
| AK45 serial + gears | 0.283 (0.44) | 0.071 | 20 | 102 | 362 | 232 | 7.0 / 2.8 | 19.2 | 16 |
| AK45 differential + gears | 0.328 (0.50) | 0.013 | 14 | 160 | 489 | 356 | 7.0 / 2.7 | 20.1 | 19 |
| AK45 differential + gears + flipper x1.38 | 0.337 (0.52) | 0.004 | 13 | 158 | 469 | 334 | 7.0 / 2.7 | 20.3 | 18 |
