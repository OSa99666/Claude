# Project context: plc-hub (common OPC UA server of all simulations)

User decisions (this session): ONE OPC UA server for all simulations - port 4840, all tags in the same namespace ns=2; simulations are
started separately and talk to it; the first simulation starts it and it runs until the last one has stopped; there is a single PLC
with ONE connection.  Dead simulation -> its tags default value + bad quality.  Stop delay configurable (`stop_delay_s`, default 30 s).
Shared hub package (not copied into each simulation).

- Design chosen after a prototype measurement (see README): hub = separate detached process (NOT inside the first simulation, whose exit
  would kill the PLC connection), simulations = OPC UA clients via `plchub.client.HubLink` with decoupled I/O task.
- `plchub/manifest.py` is the single tag list (92 tags, generated once from the simulations' own lists; test keeps them equal).
- Diagnostics in ns=3.  Bad-quality default values are stored directly in the address space (asyncua nulls bad values otherwise).
- Simulations: `[opcua] mode = "hub"|"standalone"`; each simulation's `opcua_server.py` has both paths (`HubNode` stand-ins make the
  existing read/write code work unchanged).  Mock PLCs: `plchub.config.plc_target(cfg)` + `plchub.mock_support.tolerate_bad_quality()`.
- run.sh of every simulation exports `PYTHONPATH=../plc-hub`; their pytest.ini has `pythonpath = . ../plc-hub`; their own tests run in
  standalone mode.
- Gotchas found: asyncua server does not keep the client's session name (patched to tell simulations from the PLC); mock PLCs crash on
  bad-quality reads (a real PLC just sees the quality); a PLC that is gone leaves the last PLC -> sim values in the hub (simulations only
  use them while a PLC client is connected).  CONFIRMED by the user on the real PLC: one CODESYS connection controls all simulations in parallel through the hub.
  Not explicitly checked: behaviour of the Data Source Manager on bad quality (dead simulation).
- Tests: `python -m pytest -q` in `plc-hub` (9 pass).  Never put passwords/tokens in the repo.
