// Global variable list "GVL_Sim"  (Application -> Add Object -> Global Variable List)
// This is the complete OPC UA interface to the simulator.
// In the Symbol Configuration: tick GVL_Sim, and set access rights:
//   simulator -> PLC (written by the sim):  Read/Write  (sim writes them)
//   PLC -> simulator:                        Read/Write  (sim only reads; Write lets you tune from UaExpert)
VAR_GLOBAL
    // ---- written by the simulator (sensor side) --------------------------
    Ball_X      : REAL;     // m, measured ball position (noisy, delayed), table centre = 0
    Ball_Y      : REAL;     // m
    TiltX_PV    : REAL;     // deg, actual table slope along X (after actuator lag)
    TiltY_PV    : REAL;     // deg
    Heartbeat   : DINT;     // counts up every sim cycle -> use for a watchdog

    // ---- written by the PLC (actuator side) -----------------------------
    TiltX_SP    : REAL;     // deg, >0 => ball accelerates towards +X (sim clamps to +-15)
    TiltY_SP    : REAL;     // deg, >0 => ball accelerates towards +Y
    Cmd_Reset   : BOOL;     // rising edge puts the ball at Reset_X / Reset_Y

    // ---- tuning knobs, re-read by the sim every 0.5 s -------------------
    Reset_X     : REAL := 0.10;   // m
    Reset_Y     : REAL := -0.08;  // m
    Mu_S        : REAL := 0.35;   // ball/table static friction
    Mu_K        : REAL := 0.25;   // ball/table kinetic friction (<= Mu_S)
    E_Wall      : REAL := 0.6;    // wall restitution: 1 = elastic, 0 = no bounce
    Mu_Wall     : REAL := 0.2;    // ball/wall friction (creates spin on oblique hits)
    Noise_mm    : REAL := 0.3;    // sensor noise, std. deviation
    Delay_ms    : REAL := 20.0;   // sensor delay
END_VAR
