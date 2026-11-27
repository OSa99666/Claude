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

Only **eight INT words** go over OPC UA (same names as the PLC's `gIo`; namespace `urn:ballplate:simulation`,
folder `Ballplate`, server name `Ballplate simulator`, string node ids such as `F_YR10`):

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

Table dimensions (size, ball radius, wall height and thickness) are in `config/sim.toml` `[table]`;
the wall thickness is purely visual, the physical wall is the table edge.
The plate stands on a **base with two servos**: a central pivot post with a ball joint under the plate centre, and a
servo horn with a connecting rod from each servo to the underside of the plate (one servo for each tilt axis).  The
horn angles are calculated from the simulated plate tilt (two-bar linkage), so the servos visibly tilt the plate; this
is only the view, the physics is unchanged.

Camera: mouse drag orbits, middle-drag pans, wheel zooms; keys: arrows pan, Shift+arrows orbit, +/- (or PageUp/Down) zoom, Home resets the view (a hint line is shown at the bottom of the window).  With **no OPC UA client connected the table levels itself**.

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

A desktop / app-menu icon for this simulation: `tools/install_desktop_launcher.sh` (extra arguments go to `run.sh`,
e.g. `--no-mock` for the real PLC).

Or step by step:

```bash
python3 -m venv .venv && . .venv/bin/activate
pip install -r requirements.txt
pytest                                    # 25 tests incl. a real OPC UA round trip
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

**What does the PLC really ask for?**  The simulator logs every subscription request of every client
(`logs/sim.log`, lines containing `CreateSubscription`, `ModifySubscription`, `CreateMonitoredItems`):

```bash
grep -E "Subscription|MonitoredItems" logs/sim.log
```

The log shows the publishing interval the client asked for, the sampling interval and queue size of each
monitored item, and what the server granted.  `ModifySubscription` requests are honoured (stock asyncua
ignores them).  Data changes are pushed when the simulator writes, so the update rate seen by a client
equals the subscription's publishing interval, whatever sampling interval it requests.

**If the PLC's client insists on a slow interval** (e.g. the CODESYS Data Source Manager asks for 1000 ms
whatever its update rate is set to), let the server serve it faster:

```bash
./run.sh --no-mock --publish-cap-ms 20      # (the default in config/sim.toml is now 20; set 0 to give each client exactly what it asks for)
```

The server then grants 20 ms instead of the requested interval (the log line says
`publishing_interval_cap_ms in effect`) and scales keep-alive/lifetime so the client's time-outs do not
change.  The client just receives notifications more often.  Whether the PLC applies them that often
depends on the client; check the values in the PLC.

## Sibling simulations

The same pattern (own launcher, config and ports, OPC UA server + Godot view) is used by `../valve-sim`
(bistable valve, OPC UA port 4841) and `../transport-sim` (conveyor sorting, port 4842); the table uses 4840,
so all of them can run at the same time.

## Status

* Verified here: all 25 tests (physics, scaling, closed loop with INT quantisation, and an asyncua
  client talking to the simulator: Int16 variables, tilt → ball motion, hold velocity/position, centring,
  walls removed → ball off table → outputs 0).
* **Confirmed by the user on real hardware:** the Godot UI, and the OPC UA link to the Raspberry Pi /
  CODESYS Data Source Manager at 50 Hz (`publishing_interval_cap_ms = 20`).
* **Not verified:** the CODESYS ST example (`codesys/*.st`) has not been compiled; the real PLC
  controller is maintained externally.
* Decisions to check: acceleration range ±9.81 m/s² (old model used 0..g); the velocity range defaults to
  ±0.4 m/s as in `prgSimulation`, but the ball is only clamped to it if you tick the box.
