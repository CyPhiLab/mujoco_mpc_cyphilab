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
| turn left / right | +0.479 / −0.474 rad/s | +0.471 / −0.465 rad/s | | | −3.4 / +1.6, +0.8 / +0.1 | |
| down from rest (`cruise_down_rest`, step 10 `max`) | 0.148 | 0.055 | 1.60 | 4.66 | +2.4 / −26.6 | −0.001 |
| pitch somersault (`cruise_up_rest`, step 15 `max`) | 0.132 up in the scored window; 0.27 rad/s pitch | | 2.17 | | −2.6 / +46.3 | +0.003 |

The −X mirrors and turns now have sprint comparisons (sprint −X 0.120,
−X−Y 0.131, −X+Y 0.112 m/s). Sprint up is 0.109 m/s (cot 6.84).

- Cruise costs 50–65% of the sprint's energy per metre, at 48–89% of its
  speed; −Y cruise is faster than the −Y sprint.
- Drift at full steady state is about the size of the sprint gaits' (the
  sprints: +X −21 mm/s lateral, +Y −33 mm/s depth). The optimizer's checks
  (cycles 3–4 after a steady start) see less, because with the legs held the
  attitude creeps over many strokes.
- **−Y climbs** (1.25 m in 20 s open loop). `cruise_mY_depth` (depth weight
  4, scoring cycles 9–12: `--warm_steady 8 --measure 4`) halves that at the
  same speed. Its saturation is 19%.
- **Held-leg vertical gaits do not hold their attitude**
  (`v2_cruise_vertical_30s.png`: 30 s open loop from rest, height and tilt).
  With the walking legs held, nothing in the objective sees a slow pitch
  rate: each scored 2-stroke window looks straight while the body keeps
  rotating. The sprint up/down gaits stay within 13° of level.
  - `cruise_up_rest` (from rest, `--warm 10`) is a **steady pitch
    somersault**, not a climb: constant pitch rate ≈ 0.27 rad/s (15°/s),
    upside down at 11.5 s, upright again at 21 s; height +0.9 m then back to
    0. Its scored window (strokes 11–12) is the climbing part of the loop
    (0.132 m/s), and the report's from-rest check (strokes 5–6) disagrees
    (0.072). Kept as a cruise pitch-rotation behaviour.
  - `cruise_down_rest` sinks 2.55 m in 20 s with 0.34 m sideways, a real
    down gait over 20 s, but tilts at about 5°/s (125° by 20 s, 170° by 30 s)
    and slows. From rest 0.128 m/s against 0.148 steady.
  - `cruise_up` step 21 (the 19° climbing diagonal) creeps over too, to 133°
    by 30 s. `cruise_down` step 21 (`max`) holds 33° and is the one stable
    held-leg vertical-ish gait (down 1.4 m and 2.3 m sideways in 20 s).
  - `cruise_mY` also pitches slowly (55° by 30 s), which is its climb.
  - Next: `v2_cruise_up_hold` / `v2_cruise_down_hold` rerun up and down
    with `--weights attitude_rate=8` (mean pitch and roll rate in the
    objective; rungs then need |pitch|, |roll rate| ≤ 0.015 rad/s) and a
    longer window (`--warm 10 --measure 4`).
- The 20 ms planner check is NaN on v2 (expected: the dcmotor is unstable at
  a 20 ms step).
