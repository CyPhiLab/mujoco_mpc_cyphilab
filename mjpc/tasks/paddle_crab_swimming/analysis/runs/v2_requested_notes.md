# Requested intermediate-twist gaits (crab_mimic `REQUEST_gaits.md`)

The crab_mimic tele-op policy uses the v2 library as its feed-forward. Between library gaits it does about 26–40% better than open loop (settled error 0.015–0.023 m/s on commands at 60% speed or midway between two gaits), and the library is now the limit. These fill the gaps.

- **List:** `library_v2_requested.txt` (name=file): every 1 ms rung of the runs and their mirrors, 83 gaits.
- **Ingest check:** all 83 pass crab_mimic's own ingest (`crab_swimming/mimic/library.build`: re-simulated at 4 ms from `state`, per-cycle displacement within 5% or 1 cm, rotation within 5% or 3°). Output: `v2_requested_ingest_check.txt`.

How each run was made:
- warm-started from the current gait;
- `--weights yaw=4,attitude_rate=8` (attitude held), with rotations weighted `yaw=8`;
- new `gait_chain.py --exact_speed`, so a slower rung counts only within tolerance;
- cruise means `--hold_legs R1,R2,L1,L2`.

| item | runs | result |
|---|---|---|
| 1 slower translations, 70% / 40% speed | `v2_slow_{mY,pY,pX,pXmY,pXpY,up,down}`, `v2_cslow_{mY,pY,pX,pXmY,pXpY,down}`; mirrors `*_mX*` | every rung met except sprint +X−Y at 70% (0.087 for 0.092 m/s, kept). `v2_slow_upecon` (economy up, slower) met neither rung |
| 2 translate + turn, ~50% speed, yaw ±0.15 / ±0.3 rad/s | `v2_tt_*`, `v2_ctt_*`; mirrors `*_mY_R*`, `*_mX_*` | all met except sprint −Y at 0.3 rad/s (0.062 for 0.070 m/s, kept). Cruise +X turning right at 0.3 has no gait |
| 3 slower rotations, 0.4 / 0.2 rad/s | `v2_rslow_{Xp,Xm,Yp}`, `v2_tslow_L`, `v2_crslow_*`, `v2_ctslow_L`; mirrors `*_Ym*`, `*_R*` | all met except cruise somersault −x at 0.2 (only 0.4) |
| 4 transient reorientations | `v2_tr_level_vertical`, `v2_tr_vertical_level`, `v2_tr_level_inverted` (`transient_opt.py`) | vertical → level is clean: 0° tilt and 0.006 rad/s after 6.7 s. Level → vertical reaches 84° and is still turning at 0.03 rad/s. Level → inverted stalls at 137° |

The transients are not limit cycles. Their npz holds the whole control sequence (`ctrl` at `dt`) and the start and end states, so they need their own ingest path.

Self-collision (crab_mimic `REQUEST_selfcollision.md`): these gaits were optimized with contacts off, and 20 of them pass same-side neighbouring legs through each other. `repolish.py --weights ...,clearance=100` re-solves those with ≥ 5 mm leg clearance (`gait_opt.leg_clearance`). Outputs go to `~/runs/mujoco_mpc_cyphilab/*-clear_*`.
