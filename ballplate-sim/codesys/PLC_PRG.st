// PLC_PRG : run in a cyclic task with a 10 ms interval  (Task configuration -> interval T#10ms)
PROGRAM PLC_PRG
VAR
    fbX, fbY    : FB_BallCtrl;
    tSimAlive   : TON;
    iHbPrev     : DINT;
    bSimAlive   : BOOL;
    // operator inputs (put them on a visualisation or write them from UaExpert)
    bRun        : BOOL := TRUE;
    rSpX, rSpY  : REAL;                 // m, ball setpoint, 0,0 = centre
    // simple circle generator
    bCircle     : BOOL;
    rPhase      : REAL;
    rRadius     : REAL := 0.10;         // m
    rPeriod     : REAL := 12.0;         // s
    rW, rSvX, rSvY : REAL;
END_VAR

// --- watchdog on the simulator heartbeat ---------------------------------
tSimAlive(IN := (GVL_Sim.Heartbeat = iHbPrev), PT := T#500MS);
iHbPrev := GVL_Sim.Heartbeat;
bSimAlive := NOT tSimAlive.Q;

// --- setpoint generation -------------------------------------------------
rW := 6.2831853 / rPeriod;
IF bCircle THEN
    rPhase := rPhase + rW * 0.01;
    rSpX := rRadius * COS(rPhase);   rSvX := -rRadius * rW * SIN(rPhase);
    rSpY := rRadius * SIN(rPhase);   rSvY :=  rRadius * rW * COS(rPhase);
ELSE
    rSvX := 0; rSvY := 0;
END_IF

// --- control -------------------------------------------------------------
fbX(bEnable := bRun AND bSimAlive, rSetpoint := rSpX, rSetpointVel := rSvX,
    rPos := GVL_Sim.Ball_X, rDt := 0.01);
fbY(bEnable := bRun AND bSimAlive, rSetpoint := rSpY, rSetpointVel := rSvY,
    rPos := GVL_Sim.Ball_Y, rDt := 0.01);

GVL_Sim.TiltX_SP := fbX.rTiltCmdDeg;
GVL_Sim.TiltY_SP := fbY.rTiltCmdDeg;
