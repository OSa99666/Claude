# Project context: valve simulation (standalone)

Second standalone simulation, same pattern as `../ballplate-sim` (read its CLAUDE.md for the shared
findings, e.g. the CODESYS 1000 ms publishing-interval issue and `publishing_interval_cap_ms`).
Goal: a number of independent simulations, each with its own launcher, startable from the desktop.
A third one (complex: sorting items off a conveyor) is planned; the user will supply its PLC library.

- `valvesim/valve.py` model after fbValveSimulator (source: user's OSaWareHvl library XML, uploaded in chat;
  `fbEucTwoStatesAni` itself was not included, so the behaviour is inferred from its parameters).
- OPC UA: 4 Bool vars V_YH/V_YL (PLC->sim), V_XGH/V_XGL (sim->PLC), port 4841, ns `urn:valvesim:simulation`.
  Ports differ from the table (4840, UDP 9870/9871); valve uses UDP 9872/9873.
- `valvesim/subscription_tweaks.py` is a COPY of the table's (simulations stay standalone); keep in sync if fixed.
- Godot scene (`godot/main.gd`) was written without Godot available: untested.  Needs user feedback.
- Original user IO note: V_XGH/V_XGL were both written "%I20.0" and V_YH/V_YL "%Q20.0" (probably 20.0/20.1).
- Tests: `python -m pytest -q` in `valve-sim` (7 pass).  Never put passwords/tokens in the repo.
