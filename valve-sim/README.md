# Valve simulation - PLC control demo

A bistable (double-solenoid) valve, simulated in Python and shown in Godot.  A **PLC** (CODESYS)
connects over OPC UA, like in the rolling-table demo (`../ballplate-sim`).  This is a **standalone**
simulation: own launch file, own config, own ports - it can run next to the other simulations.

```
 PLC (OPC UA client)             Linux PC
┌──────────────────┐  OPC UA   ┌─────────────────────────┐  UDP 9872 state   ┌──────────────────────┐
│ CODESYS          │──────────►│ Python simulator        │──────────────────►│ Godot: 3D view + UI  │
│ Data Source Mgr  │ port 4841 │ valve model, server     │◄──────────────────│ parameters, commands │
└──────────────────┘           └─────────────────────────┘  UDP 9873 UI      └──────────────────────┘
```

## IO (OPC UA, namespace `urn:valvesim:simulation`, folder `Valve`, string node ids, all Bool)

| Variable | Direction | Meaning |
|---|---|---|
| `V_YH` | PLC → sim | command High (active / open) |
| `V_YL` | PLC → sim | command Low (passive / closed) |
| `V_XGH` | sim → PLC | valve is High |
| `V_XGL` | sim → PLC | valve is Low |

Both feedbacks are false while the valve travels.

## Valve model (`valvesim/valve.py`, after the library block `fbValveSimulator`)

Position 0 (Low) .. 1 (High).  Bistable: the last command is remembered, with both set the previous
command stays.  A command moves the valve after an **action delay** (leaving Low: `PADFL`, leaving High:
`PADFH`, default 0.5 s) and then linearly in the **transit time** (`PTDTH`/`PTDTL`, default 3 s).
Reversing in mid-travel has no delay.  *Travel to end*: a started stroke completes even if the command is
removed.  With no OPC UA client connected, commands are ignored (the valve stays where it is).

All parameters are in the Godot panel (defaults in `config/valve.toml`): transit times, action delays,
travel-to-end, **safeguard High/Low** (`FSH`/`FSL`: forces the valve to that state, commands ignored; Low wins), **force feedbacks**
true/false (`FH`/`FL`, for fault injection; false wins) and *Set to Low / Set to High* buttons.

## View controls

Camera: mouse drag orbits, middle-drag pans, wheel zooms; keys: arrows pan, Shift+arrows orbit, +/- (or PageUp/Down) zoom, Home resets the view (a hint line is shown at the bottom of the window).

## Quick start

```bash
./run.sh                 # simulator + mock PLC (opens/closes the valve) + Godot; creates .venv on first run
./run.sh --no-mock       # real PLC: PLC connects to opc.tcp://<this-pc>:4841
./run.sh --no-godot      # headless
./run.sh --godot ~/godot/Godot_v4.3-stable_linux.x86_64
tools/install_desktop_launcher.sh      # desktop / app-menu icon (extra arguments go to run.sh)
pytest                                  # 9 tests incl. a real OPC UA round trip
```

Connecting CODESYS: same procedure as for the rolling table (`../ballplate-sim/codesys/README.md`), with
the Bool variables above and port 4841.  `publishing_interval_cap_ms = 20` in `config/valve.toml` serves
a client that insists on 1000 ms faster.

## Status

* Verified here: unit tests of the valve model, OPC UA round trip (Bool variables, open/close,
  force feedbacks, UI buttons), headless `run.sh` with the mock PLC.
* **Not verified:** the Godot scene (no Godot in the development container) and the real PLC link.
* Assumptions to check against `fbEucTwoStatesAni` (not in the supplied XML): behaviour with both commands,
  the forced-feedback precedence, the forced-feedback semantics.  The `YG` position output of the
  library block is not an OPC UA variable (only the four Bools are); Godot shows the position.
