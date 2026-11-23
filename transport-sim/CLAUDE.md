# Project context: transport / sorting simulation (standalone)

Third standalone simulation, same pattern as `../ballplate-sim` and `../valve-sim` (read their CLAUDE.md for the
shared findings: CODESYS 1000 ms publishing-interval issue and `publishing_interval_cap_ms`, run.sh/launcher pattern).

- Source: user's library XML (`fbTransportSimulator`, `tTransportSimulatorPef`), uploaded in chat (not in the repo).
  `fbEucTwoStates` itself was not included; `transportsim/twostate.py` is a COPY of valve-sim's `valve.py`
  (motors: start delay; ejectors: stroke + action delays; safeguards FSH/FSL = FORCE that state).  Keep in sync.
- OPC UA: 46 variables (see README / `transportsim/io.py`), port 4842, ns `urn:transportsim:simulation`;
  UDP 9874/9875.  Item id word T_XCQ is UInt32.  Only PLC IO is on OPC UA; feed/force/timing are Godot UI values.
- `transportsim/transport.py`: model.  Deliberate deviation: ejector barrier = queue (library's flag logic overlaps items).
- `transportsim/mock_plc.py`: sorter tracking item types from the detector queue; works at the default speeds
  (T1 raw 24000); dense traffic (items stacked 60 ticks apart) can confuse it - it is only a mock.
- Godot scene (`godot/main.gd`) was written without Godot available: untested, needs user feedback.
- Tests: `python -m pytest -q` in `transport-sim` (8 pass).  Never put passwords/tokens in the repo.
