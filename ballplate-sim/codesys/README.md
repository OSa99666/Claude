# CODESYS side (Raspberry Pi)

The OPC UA server is the **common hub** of all simulations (`../../plc-hub`, `opc.tcp://<pc-ip>:4840`, started by the first
simulation that runs).  The PLC connects to it as an
OPC UA *client*, for example through the CODESYS **Data Source Manager**, and maps the simulator's
variables onto the IO words in `gIo`.

## Variables exposed by the simulator

Namespace URI `urn:plcdemo:simulation` (ns=2 for all simulations), folder `Simulations/Ballplate`,
node ids are the string names below.  All variables are **Int16 (INT)**.

| Variable | Direction | Raw range | Meaning |
|---|---|---|---|
| `F_YR10` | PLC -> sim | +-32000 = +-15 deg | Tilt X (>0: ball accelerates towards +X) |
| `F_YR20` | PLC -> sim | +-32000 = +-15 deg | Tilt Y (>0: ball accelerates towards +Y) |
| `F_XG10` | sim -> PLC | +-32000 = +-9.81 m/s^2 | Acceleration X (negative allowed) |
| `F_XG11` | sim -> PLC | -32000..32000 = vel_min..vel_max (default +-0.4 m/s) | Velocity X |
| `F_XG12` | sim -> PLC | +-32000 = +-1 m | Distance X (table centre = 0) |
| `F_XG20`, `F_XG21`, `F_XG22` | sim -> PLC | as above | Acceleration / velocity / distance Y |

The velocity range is edited in the Godot UI; use the same range in the PLC scaling.
If the ball is off the table (walls removed) all six `F_XG..` words are 0.
Everything else (elasticity, friction, forced velocity/position, centring the ball) is set in the
Godot UI, not over OPC UA.

## Connecting

1. Start the simulator on the PC (`./run.sh`, which also starts the hub if no other simulation did).  Make sure TCP 4840 is reachable from the Pi
   (firewall: `sudo firewall-cmd --add-port=4840/tcp` on Fedora, `sudo ufw allow 4840/tcp` on Ubuntu).
2. In CODESYS add the OPC UA connection in the Data Source Manager, endpoint `opc.tcp://<pc-ip>:4840`,
   security *None* / anonymous for the first test (set `username`/`password` in `config/sim.toml` to require a login),
   and map the eight variables to `gIo.F_YR10 ... gIo.F_XG22`.
3. With **no PLC connected** the simulator keeps the table level (fail-safe).  One connection serves all simulations running on the hub;
   the tags of a simulation that is not running show their default value with bad quality.

`PLC_PRG_Example.st` + `FB_BallCtrl.st` are a small PD controller using these words.  They have not been
compiled in CODESYS; the same algorithm (`CascadePD` in `ballplate/mock_plc.py`) is tested against the simulator.

## Update rate: subscription time vs. DatasourcesTasks cycle (important)

Found on the real PLC: the CODESYS Data Source Manager only keeps the subscription time you configure if it
matches the cycle time of the **DatasourcesTasks** task.  On a mismatch it silently falls back to a **1000 ms**
update rate (it then requests 1000 ms publishing/sampling from the server, queue size 1).  A subscription time
of 40 ms needs a DatasourcesTasks cycle of 10 ms.  Symptom: the feedback words arrive about once per second
even though the simulator and the network are fast (worst when many values change at once, e.g. a circular
setpoint on both axes).

* Set the DatasourcesTasks cycle time and the connection's update/subscription time as a matching pair
  (subscription time = a multiple of the task cycle, e.g. 40 ms with 10 ms).
* The simulator's `publishing_interval_cap_ms = 20` only helps a client that asks for something slower; with
  matching CODESYS settings the client asks for its real interval and gets exactly that.
* To see what the PLC really asks for: `grep -E "Subscription|MonitoredItems" logs/sim.log`.  `logs/sim.log` also
  shows every 2 s the simulator timing, how often the PLC changes the tilt words (`PLC tilt words ...`) and
  whether the PLC's client keeps up (`OPC UA publish: ... publish cycles starved ...`).
