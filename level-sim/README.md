# Level simulation - PLC control demo (SISO, order >= 3)

A vessel filled through a valve, simulated in Python and shown in Godot.  A **PLC** (CODESYS) connects over OPC UA
like in the other demos (`../ballplate-sim`, `../valve-sim`, `../transport-sim`).  **Standalone**: own launcher,
config and ports (OPC UA 4843, UDP 9876/9877).  It re-creates the level process of the existing CODESYS simulation
(valve, delay, 2nd-order system, accumulator, random oscillations / consumption / noise) as a proper model, so the
same controller exercises can be done against it.

```
 PLC (OPC UA client)             Linux PC
┌──────────────────┐  OPC UA   ┌─────────────────────────┐  UDP 9876 state   ┌──────────────────────┐
│ CODESYS          │──────────►│ Python simulator        │──────────────────►│ Godot: 3D view,      │
│ Data Source Mgr  │ port 4843 │ process model, server   │◄──────────────────│ scope, parameters    │
└──────────────────┘           └─────────────────────────┘  UDP 9877 UI      └──────────────────────┘
```

## IO (OPC UA, namespace `urn:levelsim:simulation`, folder `Level`, string node ids)

| Variable | Type | Direction | Meaning |
|---|---|---|---|
| `T_FCV10` | Int16 | PLC → sim | inlet valve command, raw `0..32000` = 0..100 % (clamped; negative = 0) |
| `T_LT10` | Int16 | sim → PLC | level transmitter, raw `0..32000` = 0..100 % (includes the measurement noise) |
| `T_LSH10` | Bool | sim → PLC | high level switch: true when the level has **reached** the high mark (default 80 %), released below mark - hysteresis |
| `T_LSL10` | Bool | sim → PLC | low level switch: true when the level has **reached** the low mark (default 20 %, i.e. level <= mark); option *LSL inverted* makes it true while the level is above the mark (a "covered" sensor) |
| `T_LSHH10` | Bool | sim → PLC | overflow: true while the open vessel spills over the rim (level at 100 % and inflow > outflow; held 0.2 s) |

CODESYS addresses given by the user: `T_FCV10 %Qw258`, `T_LT10 %Iw258`, `T_LSH10`/`T_LSL10` both written `%I60.0`
(presumably `%I60.0` and `%I60.1`); `T_LSHH10` is new (suggest `%I60.2`).  `raw_full = 32000` is in `config/level.toml` `[scaling]`.  With no PLC connected the
valve is closed (the manual override in the panel still works).

## Process model (`levelsim/process.py`)

```
u (valve 0..100 %) -> dead time -> [valve actuator lag] -> 2nd-order block -> inflow
level h (0..100 %, integrator)  : dh/dt = k_acc * (inflow - outflow),   outflow = outlet valve + random consumption
```
`G(s) = exp(-s*delay) * gain * k_acc * wn^2 / ( s * (s^2 + 2*zeta*wn*s + wn^2) * (t_act*s + 1) )`

* Order **3** (2nd-order block + accumulator) plus dead time; a non-zero actuator lag makes it **4**.
* The level is an **integrator**, so it holds any value when inflow = outflow; the 2nd-order resonance and the dead
  time limit the controller gain.  Defaults (fn 0.15 Hz, zeta 0.3, delay 1.5 s, k_acc 0.1 /s): a P controller starts to
  oscillate around Kp ~ 5 (valve % per level %), a PI controller Kp 2, Ti 100 s settles in about a minute.
* The vessel is **open at the top**: at 100 % (`T_LT10` = 32000, `T_LSH10` true) the surplus spills over the rim.  The level stays at 100 %,
  the spill flow and the total spilled volume are shown in the HUD (red line) and as water running down the wall plus a growing puddle;
  *Set level* clears the puddle.  The level cannot go below 0 %.  No windup inside the model.  The simulator logs the transfer function and the poles at start.
* Test signals (gain in %, 0 = off): *system oscillation* (sinusoid at the input of the 2nd-order block - excites its
  resonance when its frequency equals fn), *random oscillation* (sinusoid with random frequency/amplitude added to the
  inflow), *random consumption* (band-limited random on the outflow), *measurement noise* (only on `T_LT10`).
  `seed` in the config makes the random signals repeatable.
* Actions in the panel: set the level directly, a consumption pulse (step disturbance), manual valve override
  (open-loop step responses without a PLC), a setpoint reference line for the scope (the simulator does not know the PLC setpoint).

## Exercise idea (as requested)

1. **On/off controller on the binary feedbacks**: open the valve (e.g. 60 %) when `T_LSL10`, close it when `T_LSH10`, keep
   the last state in between.  Dead time and the 2nd-order block make the level overshoot the marks (limit cycle) - watch the scope.
2. Continuous level control on `T_LT10` (P, PI, PID); find the critical gain, add noise and disturbances.
3. Switch the actuator lag on (order 4) or raise the dead time and re-tune.

## Quick start

```bash
./run.sh                 # simulator + mock PLC (on/off on the switches) + Godot; creates .venv on first run
./run.sh --mode pi --setpoint 60     # mock PLC as PI controller on T_LT10
./run.sh --no-mock       # real PLC: PLC connects to opc.tcp://<this-pc>:4843
./run.sh --no-godot      # headless
tools/install_desktop_launcher.sh      # desktop / app-menu icon (extra arguments go to run.sh)
pytest                                  # 13 tests incl. a real OPC UA round trip
```

Camera: mouse drag orbits, middle-drag pans, wheel zooms; keys: arrows pan, Shift+arrows orbit, +/- zoom, Home resets.

CODESYS: same procedure as for the rolling table (`../ballplate-sim/codesys/README.md`).  The subscription time must match the
DatasourcesTasks cycle (40 ms needs a 10 ms task), otherwise the Data Source Manager falls back to 1000 ms;
`publishing_interval_cap_ms = 20` in `config/level.toml` serves a client that insists on 1000 ms faster.

## Status

* Verified here: process model tests (order/poles, balance, integrator rate, dead time, resonance, switches, noise, closed loops),
  OPC UA round trip, headless `run.sh` with the mock on/off controller.
* **Not run in Godot** (no Godot in the development container) - the 3D view, scope and panel have to be checked by the user.
* Not verified: the real PLC link.
