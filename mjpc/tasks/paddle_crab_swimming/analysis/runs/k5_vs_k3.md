| primitive | direction | stroke (s) | speed (m/s) | yaw rate (rad/s) | lateral drift (m/s) | depth drift (m/s) | saturated | from rest: speed / yaw | 20 ms planner: speed / yaw | 20 s open loop: along, lateral, vertical (m), heading change (rad) |
|---|---|---|---|---|---|---|---|---|---|
| k3 | [0.0, -1.0, 0.0] | 1.10 | +0.1121 | +0.003 | +0.0081 | -0.0064 | 0.088 | +0.1122 / +0.005 | +0.1156 / +0.013 | +2.15, +0.12, -0.10, +0.11 |
| k5 | [0.0, -1.0, 0.0] | 1.10 | +0.1161 | -0.000 | +0.0013 | -0.0035 | 0.072 | +0.1160 / +0.001 | +0.1204 / +0.011 | +2.23, +0.02, -0.04, +0.03 |
