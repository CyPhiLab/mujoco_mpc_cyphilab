# v2 behaviours beyond straight swimming: notes on `v2_behaviours_report`

New primitives for a sequencer (imitation / RL) to switch between, on robot
v2. Sprint means all legs; cruise means the walking legs held
(`--hold_legs R1,R2,L1,L2`). Each run's final gait is `step*_max_dt0.001.npz`
if it has one, else its last `step*_dt0.001.npz`.

The 30 s open loop from rest is the judge here. A held-leg gait's attitude
can creep over many strokes while each scored window still looks right
(`v2_cruise_report_notes.md`). Report entries marked `@AX,AY` start from
rest turned to that attitude (about world x, then y), the way they were
optimized and will be used.

## Work as intended

| primitive | run | what it does (30 s open loop from rest) |
|---|---|---|
| hover / station-keep | `v2_hover` | within 8 cm of its start over 30 s, no yaw, tilt ≤ 10°; stroke 3.0 s |
| forward somersault in place | `v2_rotXp` | 0.58 rad/s about body x: 11.6 rad (≈ 1.8 turns) in 20 s, position held ±4 cm |
| backward somersault in place | `v2_rotXm` | −0.48 rad/s about body x: −9.5 rad in 20 s, held ±5 cm |
| roll in place | `v2_rotYp` | 0.61 rad/s about body y: 12.2 rad (≈ 1.9 rolls) in 20 s, held ±5 cm; its mirror rolls the other way |
| left arc | `v2_arcL_mY` | 0.156 m/s at 0.144 rad/s yaw (≈ 1.1 m radius), 2.8 rad turned in 20 s, tilt ≤ 17° |
| upside-down swim | `v2_inverted_pY` (`@3.14159,0`) | started inverted: 2.7 m in 20 s, stays inverted (170–177°). Like the vertical case, it is close to the −Y sprint stroke in a turned frame; a roll or flip has to get it there |
| cruise −Y, started vertical | `v2_cruise_up_vert_m` (`@-1.5708,0`; report name `cruise_mY_vertical`) | 0.157 m/s along body −Y = world up, cot 2.30, tilt 91–93° for 30 s. It is the cruise −Y stroke, not a climb from level |
| cruise down | `v2_cruise_down_hold2` | 0.10–0.11 m/s at cot 1.23: tilts until the body is vertical (≈ 90°) and holds there; 2.0 m down in 20 s, 3.1 m in 30 s, 0.34 m sideways |

- **There is no cruise up primitive yet.** `v2_cruise_up_vert_m`, started
  vertical, is cruise −Y in a turned frame (started level, the same gait
  swims −Y at the same speed). The new fact is that with the walking legs
  held, a vertical attitude stays put, as level and upside down do; a
  pitched attitude in between does not.
- **Up as a sequence** (open loop, one test): cruise somersault from level
  (`v2_cruise_up_rest`) until the body passes 88° (at 6.4 s), then cruise −Y.
  It climbs about 3.2 m in the next 20 s, but the tilt keeps creeping (98° →
  125° by 30 s) and it drifts 1.4 m sideways: the somersault's pitch rate
  carries through the switch. A sequencer with feedback should hold it.
- **Missing: transient reorientations** (pitch 90° and stop: level ↔
  vertical, level ↔ inverted). The pipeline optimizes periodic gaits only, so
  these need a finite-horizon optimizer.
- **Rotations:** every rate ladder ended at its 0.8 rad/s rung (x: 0.6 met,
  −0.6 missed; y: 0.6 met).

## Work, but wobble

- **Tight arc** (`v2_arcL2_mY`: 0.149 m/s at 0.30 rad/s, ≈ 0.5 m radius):
  its tilt swings between 29° and 83°. High cycle error (1.5).
- **Helices** (`v2_helixL_up`: climbs 1.65 m while turning 4.3 rad in 20 s;
  `v2_helixL_down`: sinks 1.5 m, 3.5 rad): tilt swings between 20° and 70°
  (precession).
- For turning gaits, the cycle residual compares the tilt in world axes,
  which turns with the heading. A fast turn therefore looks less periodic
  than it is.

## Did not do what was asked

- **Attitude-held climbs** (`v2_att_up_tilt1`, target 1 rad;
  `v2_att_mY_tilt05`, 0.5 rad, −Y): the ladders started each step from the
  previous step's steady state, which inherited the tilt. From rest they never
  reach it. Started at the target tilt, they relax to about 30° and swim
  climbing diagonals (about 42° and 25° above horizontal).
- **Cruise up from near level:**
  - `v2_cruise_up_hold` / `_hold2` (`--weights attitude_rate=8`): at most
    0.023 m/s with the pitch held.
  - `v2_up_trim` (legs freed, energy weight, from the held-leg attempt):
    stuck at 0.054 m/s with the pitch still drifting.
  - `v2_up_energy` (energy weight, from sprint up): stable, but cot only
    6.84 → 6.2–6.4.
  - Held legs cannot hold a pitched attitude, and the cheap and leg-driven
    regimes are separate basins.
- `v2_cruise_down_hold` stopped at its first rung (pitch rate 0.018 rad/s
  against 0.015); `_hold2` is the working one.
