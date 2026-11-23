"""The PLC IO list (OPC UA node names, folder ``Transport``).  Library names get the prefix ``T_``.

PLC -> simulator   Bool   T_YH11..T_YH15   start transport motor 1..5
                          T_YH21/T_YL21 .. T_YH24/T_YL24   extend / retract ejector 1..4
                   Int16  T_YR11..T_YR15   motor speed 0..32767 (= 0..100 %)
simulator -> PLC   Bool   T_XGH11..T_XGH15 motor running
                          T_XGH21/T_XGL21 .. T_XGH24/T_XGL24   ejector extended / retracted
                          T_XCQ01..T_XCQ04 detector 1..4 active (item type 1..4)
                          T_XCQ11..T_XCQ14 item in front of ejector 1..4
                          T_BQ01..T_BQ05   item in store 1..5
                   DWord  T_XCQ            item identification (ARGB colour of the item at the detector, else 0)
                   Int16  T_X11            transport 1 velocity (full scale = 10 ticks per minimum period)
"""
from __future__ import annotations

PLC_BOOL = (["T_YH1%d" % i for i in range(1, 6)] +
            [n for k in range(1, 5) for n in ("T_YH2%d" % k, "T_YL2%d" % k)])
PLC_INT = ["T_YR1%d" % i for i in range(1, 6)]
SIM_BOOL = (["T_XGH1%d" % i for i in range(1, 6)] +
            [n for k in range(1, 5) for n in ("T_XGH2%d" % k, "T_XGL2%d" % k)] +
            ["T_XCQ0%d" % i for i in range(1, 5)] + ["T_XCQ1%d" % i for i in range(1, 5)] +
            ["T_BQ0%d" % i for i in range(1, 6)])
SIM_DWORD = ["T_XCQ"]
SIM_INT = ["T_X11"]

DESCRIPTIONS = {
    **{"T_YH1%d" % i: "Start transport %d motor" % i for i in range(1, 6)},
    **{"T_YH2%d" % k: "Extend piston ejector %d" % k for k in range(1, 5)},
    **{"T_YL2%d" % k: "Retract piston ejector %d" % k for k in range(1, 5)},
    **{"T_YR1%d" % i: "Motor speed transport %d (0-32767)" % i for i in range(1, 6)},
    **{"T_XGH1%d" % i: "Transport %d motor running" % i for i in range(1, 6)},
    **{"T_XGH2%d" % k: "Piston ejector %d extended" % k for k in range(1, 5)},
    **{"T_XGL2%d" % k: "Piston ejector %d retracted" % k for k in range(1, 5)},
    **{"T_XCQ0%d" % i: "Detector %d active" % i for i in range(1, 5)},
    **{"T_XCQ1%d" % i: "Item in front of ejector %d" % i for i in range(1, 5)},
    **{"T_BQ0%d" % i: "Item in store %d" % i for i in range(1, 6)},
    "T_XCQ": "Item identification", "T_X11": "Transport 1 velocity",
}
