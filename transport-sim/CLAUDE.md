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
- Godot scene (`godot/main.gd`): confirmed working by the user (written without Godot in the dev container, so the
  user checks visual changes).  Decisions made with the user: motors are cylinders (T1 motor beside conveyor 1);
  item-in-front sensors hang above the ejection positions; the gate detector lamp takes the item colour (off for
  untreated type 5, which has no detector bit and runs to store 1 - intended); all detectors/motors show the metal
  colour (`METAL`) when off; ejector rod and housing are 0.5 m so the piston stays inside when extended.
- Not verified: the real PLC link / CODESYS program for this simulation; `ejector_travel_to_end` assumed true.
- Tests: `python -m pytest -q` in `transport-sim` (8 pass).  Never put passwords/tokens in the repo.
- Motors: running colour = the type-2 item green (`_tc(2)`), metal when stopped.
- View (latest): hopper + ramp at the T1 start (visual-only mapping of the first item length), items tip into open bins at the
  belt ends (the sim sends `store_in` arrival counters + `colours`; Godot animates the fall), item/lamp colours come from the
  ARGB words in the state (`colours`, `T_XCQ`).  Written WITHOUT being run in Godot (download of a Godot binary into the dev
  container was blocked), so the user must check it.
- Naming convention (user): OPC UA folder and server name follow the project (`Ballplate`/`Ballplate simulator`, `Valve`/`Valve simulator`, `Transport`/`Transport simulator`).
- Godot camera (all three, identical code block at the end of `godot/main.gd`): arrows pan, Shift+arrows orbit, +/- zoom, Home reset,
  middle-drag pan.  Keys are only used if they reach `_unhandled_input` (a focused UI field keeps them).  Not run in Godot yet - user checks.
- CODESYS: subscription time must match the DatasourcesTasks cycle (40 ms needs 10 ms) or the DSM falls back to 1000 ms (see ballplate-sim/CLAUDE.md).
- LEDs (like the valve): per motor command + running LED sitting directly on top of the motor, per ejector 2x2 LEDs directly on the housing at its back end (commands row, feedbacks row), green = High, yellow = Low, metal = off;
  `_led_node/_led/_caption/_add_motor_leds/_add_ejector_leds` in `godot/main.gd`.  Not run in Godot - user checks.
