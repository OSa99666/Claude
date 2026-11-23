# Project context: valve simulation (standalone)

Second standalone simulation, same pattern as `../ballplate-sim` (read its CLAUDE.md for the shared
findings, e.g. the CODESYS 1000 ms publishing-interval issue and `publishing_interval_cap_ms`).
Goal: a number of independent simulations, each with its own launcher, startable from the desktop.
The third one (conveyor sorting) is `../transport-sim`.

- `valvesim/valve.py` model after fbValveSimulator (source: user's OSaWareHvl library XML, uploaded in chat;
  `fbEucTwoStatesAni` itself was not included, so the behaviour is inferred from its parameters).
- OPC UA: 4 Bool vars V_YH/V_YL (PLC->sim), V_XGH/V_XGL (sim->PLC), port 4841, ns `urn:valvesim:simulation`.
  Ports differ from the table (4840, UDP 9870/9871); valve uses UDP 9872/9873.
- `valvesim/subscription_tweaks.py` is a COPY of the table's (simulations stay standalone); keep in sync if fixed.
- Godot scene (`godot/main.gd`): butterfly valve in a pipe; confirmed working by the user.  Disc and red lever bar turn
  together (same direction, bar parallel to the disc plane); animated flow particles whose speed follows sin(opening).
  Written without Godot in the dev container, so visual changes can only be checked by the user.
- Original user IO note: V_XGH/V_XGL were both written "%I20.0" and V_YH/V_YL "%Q20.0" (probably 20.0/20.1).
- Tests: `python -m pytest -q` in `valve-sim` (9 pass).  Never put passwords/tokens in the repo.
- Safeguard FSH/FSL = FORCE to that state (confirmed by how the transport library uses them: FSH21 'Force extend', FSL11 'Force stop').
- Not verified: the CODESYS side / real PLC link for this simulation; `fbEucTwoStatesAni` precedence of force-true vs force-false is assumed (false wins).
- The Valve class (`valvesim/valve.py`) is copied to `../transport-sim/transportsim/twostate.py`; keep both in sync.
- Colours (latest, per user): dimmed palette shared with transport-sim - water dark blue 0x000065, disc dark yellow
  0x656500, lever dark red 0x650000, metal (0.6,0.63,0.68), particles soft light blue.  Constants at the top of
  `godot/main.gd`.  Not run in Godot after this change (user checks visuals).
