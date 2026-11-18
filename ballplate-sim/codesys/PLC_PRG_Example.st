// Example PLC program for the rolling-table simulator (task interval 10 ms).
// Uses the IO words from the project's GVL "gIo":
//   outputs  F_YR10, F_YR20 : INT   tilt X / Y   (+-32000 <-> +-15 deg)
//   inputs   F_XG12, F_XG22 : INT   distance X/Y (+-32000 <-> +-1 m)
//            F_XG11, F_XG21 : INT   velocity X/Y (-32000..+32000 <-> vel_min..vel_max, default +-0.4 m/s)
//            F_XG10, F_XG20 : INT   acceleration (+-32000 <-> +-9.81 m/s^2)
// The ranges must match [scaling] in config/sim.toml of the simulator.
PROGRAM PLC_PRG_Example
VAR
    fbX, fbY    : FB_BallCtrl;
    bRun        : BOOL := TRUE;
    rSpX, rSpY  : REAL;                  // m, ball setpoint (0,0 = centre)
    rPosX, rPosY: REAL;                  // m, scaled measurements
    rTiltX, rTiltY : REAL;               // deg, controller outputs
END_VAR

// raw -> engineering units
rPosX := -1.0 + (INT_TO_REAL(gIo.F_XG12) + 32000.0) / 64000.0 * 2.0;
rPosY := -1.0 + (INT_TO_REAL(gIo.F_XG22) + 32000.0) / 64000.0 * 2.0;

fbX(bEnable := bRun, rSetpoint := rSpX, rSetpointVel := 0.0, rPos := rPosX, rDt := 0.01);
fbY(bEnable := bRun, rSetpoint := rSpY, rSetpointVel := 0.0, rPos := rPosY, rDt := 0.01);
rTiltX := fbX.rTiltCmdDeg;
rTiltY := fbY.rTiltCmdDeg;

// engineering units -> raw (limited to the INT range)
gIo.F_YR10 := REAL_TO_INT(LIMIT(-32000.0, rTiltX / 15.0 * 32000.0, 32000.0));
gIo.F_YR20 := REAL_TO_INT(LIMIT(-32000.0, rTiltY / 15.0 * 32000.0, 32000.0));
