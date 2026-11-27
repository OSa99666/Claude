# Jacketed water tank - MPC test plant

A heated, jacketed water tank with **level and temperature** as process values and **heater power and inflow** as control values: a 2x2
MIMO plant for the PLC's **model predictive controller (MPC)**.  The model is a Python port of the CODESYS block `fbSsHeatedJacketTank`
(state space, 4 states).  Like the other simulations it joins the common OPC UA server of `../plc-hub` (port 4840, ns=2); a Godot 3D
view shows the tank, jacket and heater, with a scope and a panel with test aids.

```
 PLC (OPC UA client, runs the MPC)         Linux PC
┌──────────────────┐  OPC UA   ┌─────────────────────────┐  UDP 9880 state   ┌──────────────────────┐
│ CODESYS          │──────────►│ Python simulator        │──────────────────►│ Godot: 3D view,      │
│ Data Source Mgr  │ port 4840 │ state-space model, hub  │◄──────────────────│ scope, test aids     │
└──────────────────┘           └─────────────────────────┘  UDP 9881 UI      └──────────────────────┘
```

## IO (OPC UA, namespace `urn:plcdemo:simulation` = ns=2 on the common hub, folder `Simulations/JacketTank`, string node ids, all Int16)

| Tag | Direction | Raw 0..32767 means | Model quantity |
|---|---|---|---|
| `M_YR01` | PLC → sim | 0..100 % | control value **heater power** u1 (limited 0..100 %) |
| `M_YR02` | PLC → sim | 0..1.0 | control value **level / inflow** u2 (limited 0..1) |
| `M_XG01` | sim → PLC | 20..120 °C | process value **temperature** y1 = x1 |
| `M_XG02` | sim → PLC | 0.1..0.9 m | process value **level** y2 = x3 |

Scaling as in the CODESYS block (`PXHR/PXLR`, `PYHR/PYLR`, raw range 0..32767); values outside the range are clamped.
The addresses supplied for the PLC are `M_YR01`/`M_YR02` both `%QW260` and `M_XG01`/`M_XG02` both `%IW260` - presumably
`%QW260`/`%QW261` and `%IW260`/`%IW261`; the simulation identifies the tags by name, the addresses only matter in the PLC.

## Model (`jacketsim/model.py`, defaults of the CODESYS block)

`x(k+1) = clamp(A x(k) + B u(k) + D d(k))`, `y = C x`, one step per sample time **PTSDT = 1 s** (the outputs hold between the steps).

```
A = [0.92   0.05  -0.0003  0.03 ]    B = [ 0.16  -0.05]    D = [-0.02   0.003]    C = [1 0 0 0]
    [0.04   0.88   0       0    ]        [ 0.01   0   ]        [-0.002  0    ]        [0 0 1 0]
    [0      0      0.95    0    ]        [ 0      0.1 ]        [ 0     -0.05 ]
    [0      0      0       0.9  ]        [ 0      0   ]        [ 0      0    ]
```
States `x1` water temperature, `x2` heater/jacket wall temperature, `x3` level, `x4` an inlet/ambient temperature state (decays from 20 °C
to its lower limit 4 °C).  Limits `XkMa = [120, 120, 0.9, 120]`, `XkMi = [2, 4, 0, 4]`; initial state `[70, 60, 0.5, 20]`.
Inputs: `u1` heater [%] (raw/32767 * 100), `u2` inflow [0..1].  Disturbances `d1` heating, `d2` level (+ drains; steady level = 2*u2 - d2),
the LREAL inputs `XD01`/`XD02` of the block, here set in the Godot panel.  Operating point: about **26 % heater and inflow 0.25 hold 69 °C
and 0.5 m**.  Both loops interact (the inflow cools the water, the level changes the heat balance a little).

**Assumptions to confirm (compare step responses with your CODESYS simulation):**
* The matrices are read from the XML as row-major initial values; the Kalman filter `fbMpcKalmanFilter` and `fcStateSpaceModel` were not
  supplied.  In the block the filter is fed with the model's own previous output, so the plant is taken as the plain deterministic model.
* `y(k+1) = C x(k+1)`: the raw outputs show the state after the step, updated at the sample instants only.
* With no PLC connected both controls are 0 (heater off, no inflow) unless the manual override is on.
* (In the ST: `tbStateMaximum := tRetValue.bStateMinimum` looks like a typo - only used internally.)

## Test aids (Godot panel; all neutral by default)

Disturbances d1 / d2 (and a d2 pulse); **manual control** of heater and inflow (open-loop step tests, e.g. for identification) overriding the PLC;
measurement noise, offset and freeze; actuator dead time; heater limit (fault); **plant-model mismatch** (heater gain, inflow gain, plant
speed factor) to test the MPC's robustness; reset to the initial state or set temperature/level directly; hold the plant; sample time PTSDT.
The scope shows temperature, level, heater and inflow over 5 minutes with dashed UI-only setpoint lines (the simulation does not know the PLC's
setpoints).

## Quick start

```bash
./run.sh                 # simulator + mock PLC + Godot; creates .venv on first run
./run.sh --temp-sp 80 --level-sp 0.6 --steps 120    # mock PLC setpoints; toggles +10 degC / +0.1 m every 120 s
./run.sh --no-mock       # the real PLC (your MPC) connects to opc.tcp://<this-pc>:4840 (the common hub)
./run.sh --no-godot      # headless
tools/install_desktop_launcher.sh      # desktop / app-menu icon (extra arguments go to run.sh)
pytest                                  # 9 tests incl. a real OPC UA round trip
```

The mock PLC is two simple PI loops (temperature -> heater, level -> inflow), **not** an MPC; the real MPC runs in the PLC.
`[opcua] mode = "standalone"` gives the simulation its own server (tests).  See `../plc-hub/README.md` for the common OPC UA server.

## Status

* Verified here: model tests (hand-calculated first samples, steady state against an independent linear solution, limits, disturbances,
  all test aids, two PI loops settle), OPC UA round trip, headless `run.sh` with the mock PLC (settles at 80 °C / 0.6 m in about 40 s),
  six real simulations on the hub.
* **Not run in Godot** (no Godot in the development container) - the 3D view, scope and panel have to be checked by the user.
* Not verified: the model against your CODESYS simulation numbers, and the real PLC link with your MPC.
