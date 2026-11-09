# CODESYS side (Raspberry Pi)

CODESYS Control for Raspberry Pi SL runs the controller and acts as the **OPC UA server**.
The simulator connects as a client and exchanges the variables in `GVL_Sim`.

1. New standard project -> device *CODESYS Control for Raspberry Pi SL* (ST language).
2. Add a **Global Variable List** named `GVL_Sim`, paste `GVL_Sim.st` (the `VAR_GLOBAL` block).
3. Add a **Function Block** `FB_BallCtrl` (ST) and paste the file; make **PLC_PRG** from `PLC_PRG.st`
   (the `VAR` part goes in the declaration editor, the rest in the body).
   `FB_BallCtrl.st` and `PLC_PRG.st` each contain declaration and body in one file for readability.
4. Task configuration: `PlcTask` cyclic, **10 ms**, calling `PLC_PRG`.
5. Add **Symbol Configuration**, tick *Support OPC UA features*, tick `GVL_Sim` and give the variables
   Read/Write access.  Download to the Pi.
6. On the Pi's runtime enable OPC UA (default port 4840) and create a user/password or allow anonymous
   for a first test.  Make sure TCP 4840 is open.
7. Find the node ids: `python -m tools.browse_nodes opc.tcp://<pi-ip>:4840` and compare with
   `node_template` in `config/sim.toml`.  Typically `ns=4;s=|var|CODESYS Control for Raspberry Pi SL.Application.GVL_Sim.Ball_X`.

Operator inputs in `PLC_PRG` (`bRun`, `bCircle`, `rSpX`, `rSpY`, `rRadius`, `rPeriod`) can be changed
online in the CODESYS debugger or written over OPC UA.  Pulse `GVL_Sim.Cmd_Reset` to drop the ball at
`Reset_X`/`Reset_Y`.  Tune `Mu_S`, `E_Wall`, ... online to see the controller cope with a slippery or
bouncy ball.

Note: this ST has not been compiled in CODESYS (no runtime available while writing it).  It mirrors
`CascadePD` in `ballplate/mock_plc.py`, which is tested.  If the compiler objects to a detail
(e.g. the `VAR CONSTANT` typing), the algorithm is the five lines at the end of `FB_BallCtrl`.
