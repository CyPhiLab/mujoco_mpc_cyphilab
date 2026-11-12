# v2 cruise gaits: notes on `v2_cruise_report`

Cruise: walking legs held (`--hold_legs R1,R2,L1,L2`), paddles only.
Steady state at 1 ms (12 strokes from rest, 2 scored), from
`primitive_report.py`; the sprint gaits are re-scored the same way.

| primitive | cruise speed (m/s) | sprint speed | cruise cot | sprint cot | cruise lateral / depth drift (mm/s) | cruise yaw (rad/s) |
|---|---|---|---|---|---|---|
| −Y (`cruise_mY`, step 14) | 0.168 | 0.141 | 2.63 | 5.38 | +1.3 / +46.9 | +0.005 |
| −Y, depth weight 4 (`cruise_mY_depth`) | 0.172 | 0.141 | 2.55 | 5.38 | −3.9 / +22.6 | −0.000 |
| +Y | 0.109 | 0.123 | 3.83 | 6.26 | −3.5 / −14.1 | −0.004 |
| +X | 0.058 | 0.119 | 3.60 | 6.60 | +18.7 / +1.6 | +0.008 |
| +X−Y | 0.088 | 0.131 | 3.46 | 5.92 | +10.5 / +0.8 | +0.001 |
| +X+Y | 0.093 | 0.112 | 4.06 | 6.24 | +25.7 / +0.5 | −0.004 |
| −X (mirror) | 0.059 | | 3.62 | | −20.4 / +3.0 | −0.008 |
| −X−Y (mirror) | 0.089 | | 3.45 | | −9.4 / +1.4 | +0.001 |
| −X+Y (mirror) | 0.095 | | 4.02 | | −22.0 / +0.8 | +0.007 |
| turn left / right | +0.479 / −0.474 rad/s | | | | −3.4 / +1.6, +0.8 / +0.1 | |

- Cruise costs 50–65% of the sprint's energy per metre, at 48–89% of its
  speed; −Y cruise is faster than the −Y sprint.
- Drift at full steady state is about the size of the sprint gaits' (the
  sprints: +X −21 mm/s lateral, +Y −33 mm/s depth). The optimizer's checks
  (cycles 3–4 after a steady start) see less, because with the legs held the
  attitude creeps over many strokes.
- **−Y climbs** (1.25 m in 20 s open loop). `cruise_mY_depth` (depth weight
  4, scoring cycles 9–12: `--warm_steady 8 --measure 4`) halves that at the
  same speed. Its saturation is 19%.
- **Up and down are diagonals, not vertical.** Their ladders started each
  step from the previous gait's steady state, where the body had pitched to
  1.0 rad; the optimizer scored ~0.15 m/s vertical in that attitude. From
  rest the body settles at 0.28 rad (up) or 0.41–0.52 rad (down):
  - `cruise_up` step 21: 0.049 m/s up and 0.142 m/s sideways (about 19°
    above horizontal), stable and straight; worth keeping as a climbing
    diagonal. Step 22 (`max`) sinks.
  - `cruise_down` step 20 / 21: 0.055 / 0.074 m/s down and 0.125 m/s
    sideways (about 25–31° below horizontal), no yaw.
  - `v2_cruise_up_rest` and `v2_cruise_down_rest` rerun them with
    `--from_rest --warm 10` (every rollout from rest, scored after 10
    strokes), warm-started from up step 21 and down step 21. Not in this
    commit; still running.
- The 20 ms planner check is NaN on v2 (expected: the dcmotor is unstable at
  a 20 ms step).
