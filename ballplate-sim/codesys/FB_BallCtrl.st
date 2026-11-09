// Function block FB_BallCtrl : one axis of the ball-and-plate controller.
// Plant is (almost) a double integrator:  x'' = (5/7) * g * theta
// so a PD law with the 5/7*g gain scaled out gives chosen wn / zeta:
//     theta = ( wn^2 * (sp - x) - 2*zeta*wn * (v_hat - sp_vel) ) / k
// Same algorithm as CascadePD in ballplate/mock_plc.py.
FUNCTION_BLOCK FB_BallCtrl
VAR_INPUT
    bEnable     : BOOL;
    rSetpoint   : REAL;                 // m
    rSetpointVel: REAL;                 // m/s velocity feed-forward (0 for fixed setpoint)
    rPos        : REAL;                 // m, measured
    rDt         : REAL := 0.01;         // s, task cycle time (use the task interval!)
    rWn         : REAL := 2.5;          // rad/s  closed-loop natural frequency
    rZeta       : REAL := 0.9;
    rVelTau     : REAL := 0.06;         // s, velocity filter time constant
    rKi         : REAL := 0.15;         // integral gain (fights rolling resistance)
    rMaxDeg     : REAL := 10.0;
END_VAR
VAR_OUTPUT
    rTiltCmdDeg : REAL;                 // deg
END_VAR
VAR
    rPosPrev    : REAL;
    rVel        : REAL;
    rInteg      : REAL;                 // rad
    bInit       : BOOL;
    rErr, rCmd, rA : REAL;
END_VAR
VAR CONSTANT
    K_PLANT     : REAL := 7.0071;       // 5/7 * 9.81
    RAD2DEG     : REAL := 57.29578;
END_VAR

IF NOT bEnable THEN
    rInteg := 0; bInit := FALSE; rTiltCmdDeg := 0; RETURN;
END_IF
IF NOT bInit THEN rPosPrev := rPos; rVel := 0; bInit := TRUE; END_IF

// filtered derivative of the measured position
rA := rDt / (rVelTau + rDt);
rVel := rVel + rA * ((rPos - rPosPrev) / rDt - rVel);
rPosPrev := rPos;

rErr := rSetpoint - rPos;
rInteg := LIMIT(-0.05, rInteg + rKi * rErr * rDt, 0.05);
rCmd := (rWn*rWn*rErr - 2.0*rZeta*rWn*(rVel - rSetpointVel)) / K_PLANT + rInteg;
rTiltCmdDeg := LIMIT(-rMaxDeg, rCmd * RAD2DEG, rMaxDeg);
