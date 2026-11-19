# Rolling table - PLC control demo (v2: simulator is the OPC UA server)

A solid ball (12 cm) rolls on a 2 m x 2 m tilting table with 12 cm walls.  A **PLC** (CODESYS on a
Raspberry Pi) controls the two tilt angles; a **Linux PC** simulates the physics and shows it in **Godot**,
which is also the user interface for the simulation parameters.

```
 PLC (OPC UA client)             Linux PC
┌──────────────────┐  OPC UA   ┌─────────────────────────┐  UDP 9870 state   ┌──────────────────────┐
│ CODESYS          │──────────►│ Python simulator        │──────────────────►│ Godot: 3D view + UI  │
│ Data Source Mgr  │ port 4840 │ physics @1 kHz, server  │◄──────────────────│ parameters, commands │
└──────────────────┘           └─────────────────────────┘  UDP 9871 UI      └──────────────────────┘
```

Only **eight INT words** go over OPC UA (same names as the PLC's `gIo`):

| Variable | Direction | Raw ±32000 means |
|---|---|---|
| `F_YR10`, `F_YR20` | PLC → sim | ±15° tilt X / Y (>0 accelerates the ball towards +X / +Y) |
| `F_XG10`, `F_XG20` | sim → PLC | ±9.81 m/s² acceleration X / Y (negative allowed) |
| `F_XG11`, `F_XG21` | sim → PLC | `vel_min`..`vel_max` (default ±0.4 m/s) velocity X / Y |
| `F_XG12`, `F_XG22` | sim → PLC | ±1 m distance X / Y (centre = 0) |

Scaling lives in `config/sim.toml` (`[scaling]`); the PLC must use the same ranges.

## Parameters and commands (Godot panel, top right)

| UI control | Meaning (old PLC name) |
|---|---|
| Elasticity | wall restitution 0..1 (`PELAC`); **negative removes the walls** - the ball can then roll off the table and falls; its `F_XG..` words read 0 |
| Static / dynamic friction | ball/table friction coefficients (`PFRS`, `PFRD`) |
| Min / max velocity | range for the raw velocity word (`PVLR`, `PVHR`); optional checkbox clamps the ball's velocity to it |
| Velocity setpoint X/Y + *Set velocity* | forces the ball velocity while checked (`PVSPX/Y`, `RX11/RX21`) |
| Distance setpoint X/Y + *Set distance* | pins the ball position while checked (`PDSPX/Y`, `RX12/RX22`) |
| **Center ball** | ball to the centre, at rest, back on the table (`RXBL`) |

Mouse: drag to orbit, wheel to zoom.  With **no OPC UA client connected the table levels itself**.

## Physics (`ballplate/physics.py`)

Solid sphere, table frame.  Ball/table Coulomb friction with static and kinetic coefficients (rolling
`a = 5/7·g·sin θ`; it slips when `tan θ > 3.5·μs`), rolling resistance, walls with normal restitution and
wall friction that couples into vertical-axis spin.  Tilt actuator: first-order lag, ±15°, 60°/s rate limit.
Setpoints/holds, velocity clamp and the falling ball are in the same file.

## Quick start

One command starts everything (creates `.venv` on first run, waits for the OPC UA port, then starts the
mock PLC and Godot; closing the Godot window or Ctrl+C stops all three; logs in `logs/`):

```bash
./run.sh                    # simulator + mock PLC (ball held at centre) + Godot
./run.sh --pattern circle   # mock PLC drives the ball around a circle (center|circle|square)
./run.sh --no-mock          # real PLC: simulator + Godot only, PLC connects to opc.tcp://<this-pc>:4840
./run.sh --no-godot         # headless, e.g. over SSH
./run.sh --godot ~/godot/Godot_v4.3-stable_linux.x86_64   # or GODOT=... ; otherwise godot, godot4,
                                                          # ~/godot/Godot_v*, or the Flatpak is used
```

Or step by step:

```bash
python3 -m venv .venv && . .venv/bin/activate
pip install -r requirements.txt
pytest                                    # 23 tests incl. a real OPC UA round trip
```

Without a PLC (a mock client plays the PLC):

```bash
python -m ballplate.sim                                  # server on opc.tcp://0.0.0.0:4840  (takes ~2 s to start)
python -m ballplate.mock_plc --pattern circle            # client: PD controller, patterns center|circle|square
godot --path godot                                       # 3D view + UI (Godot 4.2+)
```

With the real PLC: start the simulator, open TCP 4840 on the PC, then connect from CODESYS (see
[`codesys/README.md`](codesys/README.md)).  If Godot runs on another machine set `[visual] host` in
`config/sim.toml` to its address; Godot sends its UI values back to whichever address the state comes from.
`python -m tools.browse_nodes opc.tcp://<pc>:4840` lists what the server exposes.

**Checking the OPC UA update rate** (run it next to the PLC to see what the network delivers):

```bash
python -m tools.rate_check opc.tcp://<pc>:4840                    # subscription at 20 ms
python -m tools.rate_check opc.tcp://<pc>:4840 --interval 1000    # what a 1 s default gives
python -m tools.rate_check opc.tcp://<pc>:4840 --mode poll        # read loop, round-trip times
```

The simulator refreshes the words every 10 ms and the server delivers exactly the publishing interval a
client asks for (10 ms -> 100/s, 20 ms -> 50/s, 1000 ms -> 1/s).  A subscription only reports *changes*, so
keep the ball moving when measuring.  If the PLC sees a slower rate than `rate_check`, the PLC's connection
settings (publishing/sampling/poll interval, task cycle) are the limit.

## Status

* Verified here: all 23 tests (physics, scaling, closed loop with INT quantisation, and an asyncua
  client talking to the simulator: Int16 variables, tilt → ball motion, hold velocity/position, centring,
  walls removed → ball off table → outputs 0).
* **Not verified:** the new Godot UI (not run here), the CODESYS ST example (not compiled), and the
  connection from the real Pi / Data Source Manager.
* Decisions to check: acceleration range ±9.81 m/s² (old model used 0..g); the velocity range defaults to
  ±0.4 m/s as in `prgSimulation`, but the ball is only clamped to it if you tick the box.
