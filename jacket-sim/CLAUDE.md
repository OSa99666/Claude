# Project context: jacketed water tank (MPC test plant, sixth simulation)

Same pattern as the other simulations (level-sim is the closest); joins the common OPC UA hub (`../plc-hub`, port 4840, ns=2) - see the
hub's CLAUDE.md.  OPC UA folder `JacketTank` (standalone), hub folder `Simulations/JacketTank`; UDP 9880/9881; standalone endpoint 4845.

- Source: user's CODESYS library XML `JacketWaterTank.xml` (`fbSsHeatedJacketTank` = state-space simulator template with a Kalman filter,
  `fbAiAoScaling`), uploaded in chat (not in the repo).  `fbMpcKalmanFilter`, `fcStateSpaceModel`, `fbSystemIoScaling` were NOT included.
  The user's MPC (in the PLC) uses this plant; "basically a CSTR-like jacketed tank that works fine as an MPC example".
- IO as given: `M_YR01` heater control value, `M_YR02` level control value (PLC -> sim), `M_XG01` temperature, `M_XG02` level (sim -> PLC), all Int16,
  raw 0..32767 ("max is 32767").  Their %QW260/%IW260 addresses overlap (both tags of a pair); proposal %QW260/261, %IW260/261 in the README.
- Model `jacketsim/model.py`: 4 states, A/B/C/D/limits/initial state/ranges exactly the block's defaults (row-major array initial values),
  stepped once per PTSDT (1 s); KF omitted (deterministic plant; assumption to confirm); y = C x_next; outputs hold between samples.
  Test aids: d1/d2, d2 pulse, manual override, noise/offset/freeze, dead time, heater limit, gain/speed mismatch, reset/set state, hold.
- Mock PLC: two PI loops (`Controller`, tuned kp 1.5/ti 15 s heater, kp 2/ti 12 s inflow), `--temp-sp/--level-sp/--steps`; NOT an MPC.
- Godot (`godot/main.gd`): 3D tank with jacket (shell colour = wall temp, coils glow with heater power, water colour = temperature), inlet/outlet
  pipes + valves, level scale, scope (5 min, UI-only setpoint lines), scrolling panel; camera block as the others (CAM_DIST_MAX 40).
  WRITTEN WITHOUT RUNNING GODOT - the user checks it.
- Tests: `python -m pytest -q` in `jacket-sim` (9 pass).  Never put passwords/tokens in the repo.
