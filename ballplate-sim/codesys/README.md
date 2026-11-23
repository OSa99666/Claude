# CODESYS side (Raspberry Pi)

The **simulator is the OPC UA server** (default `opc.tcp://<pc-ip>:4840`).  The PLC connects to it as an
OPC UA *client*, for example through the CODESYS **Data Source Manager**, and maps the simulator's
variables onto the IO words in `gIo`.

## Variables exposed by the simulator

Namespace URI `urn:ballplate:simulation` (index differs per client - browse for it), folder `Ballplate`,
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

1. Start the simulator on the PC (`python -m ballplate.sim`).  Make sure TCP 4840 is reachable from the Pi
   (firewall: `sudo firewall-cmd --add-port=4840/tcp` on Fedora, `sudo ufw allow 4840/tcp` on Ubuntu).
2. In CODESYS add the OPC UA connection in the Data Source Manager, endpoint `opc.tcp://<pc-ip>:4840`,
   security *None* / anonymous for the first test (set `username`/`password` in `config/sim.toml` to require a login),
   and map the eight variables to `gIo.F_YR10 ... gIo.F_XG22`.
3. With **no client connected** the simulator keeps the table level (fail-safe).

`PLC_PRG_Example.st` + `FB_BallCtrl.st` are a small PD controller using these words.  They have not been
compiled in CODESYS; the same algorithm (`CascadePD` in `ballplate/mock_plc.py`) is tested against the simulator.
