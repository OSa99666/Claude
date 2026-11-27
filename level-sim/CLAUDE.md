# Project context: level process simulation (standalone, fourth simulation)

Same pattern as `../ballplate-sim`, `../valve-sim`, `../transport-sim` (read their CLAUDE.md for the shared findings: CODESYS
1000 ms publishing-interval issue / `publishing_interval_cap_ms`, run.sh + launcher pattern).  `subscription_tweaks.py`, `ui_link.py`,
`config.py` are COPIES (keep in sync).  Naming: OPC UA folder `Level`, server `Level simulator`.

- User request: a SISO control process of order >= 3, modelled on the existing CODESYS level simulation (HMI picture: valve, delay,
  system/random oscillations, 2nd-order system, accumulator with reinforcement, random consumption, random noise, H/L switches).
- IO as given by the user: `T_FCV10` Int (PLC->sim, %Qw258), `T_LT10` Int (sim->PLC, %Iw258), `T_LSH10`, `T_LSL10` Bool (sim->PLC,
  both written %I60.0 - probably 60.0/60.1).  Raw 0..32000 = 0..100 % (Int16 like ballplate).  Port 4843, UDP 9876/9877.
  The binary switches are for a first on/off controller exercise.
- Model `levelsim/process.py`: dead time -> optional actuator lag -> 2nd order -> accumulator (integrator); order 3 (4 with lag) + delay.
  Defaults chosen so a P controller goes unstable around Kp ~5 and PI 2/100 s settles.  CONFIRMED by the user:
  `T_LSH10`/`T_LSL10` are true when the level has REACHED the mark (LSL: level <= low mark); `lsl_inverted` flips LSL to "covered" (optional).
- Mock PLC (`levelsim/mock_plc.py`): `--mode onoff` (valve 60 % / 0 on the switches, limit cycle ~13..87 % with marks 20/80) or `--mode pi`.
- Godot (`godot/main.gd`): vessel, inlet/outlet pipes + valves, switch lamps, scope (level/valve/setpoint line), scrolling parameter panel,
  camera block identical to the other sims.  WRITTEN WITHOUT RUNNING GODOT - the user checks it.
- Tests: `python -m pytest -q` in `level-sim` (13 pass).  Never put passwords/tokens in the repo.
- Vessel is OPEN at the top (user request): level > 100 % spills (`spilled`, `q_over`, `overflowing` in `process.py`, state keys `q_over/spilled/overflow`); Godot shows a rim ring, wall stream, puddle, HUD line.  Elbow of the inlet pipe closed with a sphere.  Not run in Godot.
