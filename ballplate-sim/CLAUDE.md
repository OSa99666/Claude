# Project context: rolling table (ball on a tilting table) PLC demo

Saved from the working session so a later session can continue without the chat history.

## Sibling simulations
Standalone simulations in the same repo, each with its own launcher, ports and CLAUDE.md: `../valve-sim`
(bistable valve, OPC UA 4841) and `../transport-sim` (conveyor sorting, OPC UA 4842).  The table uses 4840.
They copy `subscription_tweaks.py` (and the valve/two-state model) instead of sharing code; keep copies in sync.
Decided with the user: the PLC control is external (no ST compiling here), "poke" is covered by the forced
velocity/position setpoints, tuning plots are done in CODESYS, OPC UA security is on hold (lab access via VPN).

## Goal
A PLC (CODESYS on a Raspberry Pi) controls a ball rolling on a tilting 2 m x 2 m table.  A Linux PC
simulates the physics (Python) and shows/controls it in Godot.  Replaces an older simulation that ran
inside the PLC (`fbTiltingTable`/`fbTiltingAxis`, point-mass per axis, PLCopen XML supplied by the user).

## Architecture (v2, current)
- `ballplate/physics.py`  rigid-body model: solid sphere, static/kinetic friction, rolling resistance,
  walls with restitution + wall friction (couples to vertical spin), tilt actuator (lag, +-15 deg, 60 deg/s).
  Extras: walls removable (ball falls off), forced velocity/position holds, optional velocity clamp, `center()`.
- `ballplate/sim.py`  10 ms cycle, 1 ms physics steps.  Runs the OPC UA **server**, receives UI values from
  Godot (UDP 9871), sends state to Godot (UDP 9870).  No OPC UA client connected => table levels itself.
- `ballplate/opcua_server.py`  asyncua server, namespace `urn:ballplate:simulation`, folder `RollingTable`,
  eight **Int16** variables only: `F_YR10/F_YR20` (PLC->sim tilt X/Y), `F_XG10/11/12` (accel/vel/dist X),
  `F_XG20/21/22` (Y).  Raw +-32000 scaling in `config/sim.toml [scaling]` (tilt +-15 deg, accel +-9.81 m/s2,
  dist +-1 m, velocity vel_min..vel_max default +-0.4 m/s).  The PLC must use the same ranges.
- `ballplate/subscription_tweaks.py`  logs every client subscription request; honours ModifySubscription
  (stock asyncua ignores it); optional `publishing_interval_cap_ms` (default 20) serves slow clients faster.
- `ballplate/mock_plc.py`  OPC UA *client* with a PD controller (stands in for the PLC); patterns center/circle/square.
- `godot/main.gd`  3D view + control panel (elasticity, static/dynamic friction, velocity range, forced
  velocity/position, Center ball).  Elasticity < 0 removes the walls.  Physics is NOT in Godot.
- `run.sh`  starts simulator + mock PLC + Godot (`--no-mock`, `--no-godot`, `--pattern`, `--publish-cap-ms`).
- `tools/browse_nodes.py`, `tools/rate_check.py`  inspect the server / measure real update rates.
- `codesys/`  ST controller example + notes for the CODESYS Data Source Manager (not compiled).

## Decisions made with the user
- Own rolling-ball dynamics (not the old point-mass model).  Geometry: table +-1 m, ball 12 cm diameter,
  walls 12 cm high, wall thickness visual only (`[table] wall_thickness_m`, default 0.003).
- Only the eight INT IOs are OPC UA variables; all parameters/commands are set in the Godot UI
  (old names: PELAC elasticity, PFRS/PFRD friction, PVHR/PVLR velocity range, PVSPx/PDSPx setpoints,
  RX11/RX21 set velocity, RX12/RX22 set distance, RXBL center ball).
- Acceleration word is symmetric (negative allowed).  Velocity range defaults to +-0.4 m/s (as in the old
  `prgSimulation`); the ball is only clamped to it if "Limit ball velocity" is ticked.
- The simulator is the OPC UA server; CODESYS connects as a client via the Data Source Manager.

## Findings worth remembering
- CODESYS Data Source Manager asked for a 1000 ms publishing/sampling interval whatever its update rate
  was set to (4000 ms worked, 200 ms became 1 s).  Fix: server-side `publishing_interval_cap_ms = 20`
  (now default) -> 50 Hz at the PLC.  Confirmed working by the user.
- asyncua server startup takes ~2 s; wait for the port before connecting.  Data changes are pushed when the
  simulator writes, so a client's update rate = subscription publishing interval; unchanged values send nothing.
- Scripts started in the background by a non-interactive shell ignore SIGINT (test with SIGTERM); Ctrl+C
  works in a real terminal.  Bare `pytest` works thanks to `pytest.ini` (`pythonpath = .`).
- User environment: Ubuntu with Python 3.14 and Godot 4.3 (works), Raspberry Pi with CODESYS.  xrdp/KRdp
  on Fedora KDE was unreliable; Ubuntu GNOME remote desktop works.

## Status
- 25 tests pass (physics, scaling, closed loop with INT quantisation, real OPC UA round trips,
  subscription modify/cap).  Godot UI and the OPC UA link to the real Pi were confirmed working by the user.
- Not verified: the CODESYS ST example (`codesys/*.st`) has not been compiled (reference only; control is external).

## Possible next steps
Project is feature-complete for now.  Decided with the user (latest session) NOT to do:
- ST controller compile/tuning: the PLC control is external and maintained by the user.
- Godot "poke the ball": covered by the forced velocity/position setpoints.
- Live tuning plots: done in CODESYS.
- OPC UA security: on hold; lab access is via VPN.
Open ideas only if the user asks: tests/logging for client disconnect/reconnect over the VPN.

## Working notes for a future session
- Develop on branch `claude/optimistic-ramanujan-0bcprr`; push there.  Never put passwords/tokens in the repo.
  GitHub does not accept account passwords for git over HTTPS (use a token or SSH).
- Run tests: `python -m pytest -q` in `ballplate-sim` (venv with `pip install -r requirements.txt`).
