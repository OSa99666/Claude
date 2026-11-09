# Ball on a tilting table - PLC control demo

A solid ball rolls on a square tilting table.  A **CODESYS PLC (Raspberry Pi)** controls the two tilt
angles; a **Linux PC** simulates the physics and renders it in **Godot**.  All process data goes over
**OPC UA**.

```
 Raspberry Pi                        Linux PC
┌──────────────────┐   OPC UA    ┌────────────────────────┐  UDP/JSON  ┌────────────┐
│ CODESYS runtime  │◄───────────►│ Python simulator       │───────────►│ Godot 4    │
│ PID  (server)    │  port 4840  │ physics @1 kHz (client)│  port 9870 │ 3D view    │
└──────────────────┘             └────────────────────────┘            └────────────┘
```

* The PLC is the OPC UA **server**; the simulator is the **client** (reads `TiltX_SP`, `TiltY_SP`,
  writes the sensor values).
* Physics is deliberately in Python, not Godot: fixed 1 ms step independent of frame rate, headless and
  unit-testable.  Godot only draws what it receives.

## Physics (ballplate/physics.py)

Solid sphere, table frame, tilt convention: `TiltX > 0` => ball accelerates towards **+X** (same for Y).

* Ball/table Coulomb friction with **static (mu_s)** and **kinetic (mu_k)** coefficients.  Each step
  computes the friction that would give pure rolling; if it exceeds `mu_s*N` the ball slips under
  `mu_k*N`.  Rolling gives `a = 5/7 g sin(theta)`; slipping occurs when `tan(theta) > 3.5 mu_s`.
  A ball released sliding ends up rolling at 5/7 of its speed (tested).
* Rolling resistance `c_rr`.
* Walls: restitution `e_wall` (1 = elastic ... 0 = no bounce) and wall friction `mu_wall` coupled to
  vertical-axis spin, so oblique hits leave with spin and a non-mirror angle.
* Tilt actuator: first-order lag, +-15 deg limit, 60 deg/s rate limit.
* Sensor seen by the PLC: Gaussian noise and a transport delay (both tunable from the PLC).
* Fail-safe: if the PLC link drops for more than 1 s the table levels itself.

## Quick start

```bash
python3 -m venv .venv && . .venv/bin/activate
pip install -r requirements.txt
pytest                                   # physics + closed-loop tests
```

### Without hardware (mock PLC)

```bash
python -m ballplate.mock_plc --pattern circle    # pretend CODESYS: OPC UA server + controller
python -m ballplate.sim --url opc.tcp://127.0.0.1:4840
godot --path godot                               # optional 3D view (Godot 4.2+)
```
`--pattern` is `center`, `circle` or `square`.

### With the real PLC

1. Set up CODESYS as described in [`codesys/README.md`](codesys/README.md).
2. Edit `config/sim.toml`: `opcua.url` (Pi address), `namespace`, `device`, credentials if used.
   `python -m tools.browse_nodes opc.tcp://<pi>:4840` prints the node ids.
3. `python -m ballplate.sim`, then `godot --path godot`.
   (If Godot runs on another PC set `[visual] host` to its address.)

## OPC UA variables (`GVL_Sim`)

| Name | Dir (from PLC) | Unit | Meaning |
|---|---|---|---|
| `Ball_X`, `Ball_Y` | in | m | ball position as measured (noise + delay), centre = 0 |
| `TiltX_PV`, `TiltY_PV` | in | deg | actual table slope |
| `Heartbeat` | in | - | DINT, increments each 10 ms cycle |
| `TiltX_SP`, `TiltY_SP` | out | deg | commanded slope |
| `Cmd_Reset` | out | - | rising edge: ball to `Reset_X/Reset_Y` |
| `Reset_X`, `Reset_Y` | out | m | |
| `Mu_S`, `Mu_K`, `E_Wall`, `Mu_Wall` | out | - | contact parameters, live |
| `Noise_mm`, `Delay_ms` | out | mm, ms | sensor imperfections, live |

The simulator reads the data type of each node on connect and writes with exactly that type, so
`REAL`/`LREAL`/`DINT` all work.

## Status / what was verified

* `pytest`: 14 tests pass (rolling acceleration, slip threshold, slide-to-roll 5/7, restitution,
  energy, wall spin, containment, actuator limits, and closed loop with noise and delay).
* End-to-end over real OPC UA (asyncua mock server <-> simulator): ball captured and held at the
  centre.
* **Not verified:** the Godot scene (could not run Godot in the build environment), the CODESYS ST
  code (not compiled), and the real Raspberry Pi connection.  Expect to fix small things there.

## Ideas

Replace polling with OPC UA subscriptions for `TiltX_SP/TiltY_SP`; add a keyboard "poke the ball"
disturbance in Godot; run a second controller (MPC) in the PLC; use the Pi's real-time kernel.
