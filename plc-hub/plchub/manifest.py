"""The common tag list of all simulations: the single source of truth for the hub's address space (namespace ns=2).

Every tag has a unique node id (the tag name), a type and a description; ``plc_to_sim`` tags are written by the PLC, ``sim_to_plc``
tags by the simulation.  All tags exist while the hub runs, whether the owning simulation is running or not (then they show
their default value with bad quality).  A new simulation needs an entry here; ``tests/test_manifest.py`` checks uniqueness and
that every simulation's own tag definitions match this list.
"""
from __future__ import annotations

# sim -> {"folder": folder name, "title": text, "plc_to_sim": {tag: (type, description)}, "sim_to_plc": {...}}
# types: "Boolean", "Int16", "UInt32"
MANIFEST = {
    "ballplate": {
        "folder": "Ballplate",
        "title": "Rolling table (tilting plate with a ball), PLC controls the two tilt angles",
        "plc_to_sim": {
            "F_YR10": ("Int16", "Tilt X direction"),
            "F_YR20": ("Int16", "Tilt Y direction"),
        },
        "sim_to_plc": {
            "F_XG10": ("Int16", "Acceleration X direction"),
            "F_XG11": ("Int16", "Velocity X direction"),
            "F_XG12": ("Int16", "Distance X direction"),
            "F_XG20": ("Int16", "Acceleration Y direction"),
            "F_XG21": ("Int16", "Velocity Y direction"),
            "F_XG22": ("Int16", "Distance Y direction"),
        },
    },
    "valve": {
        "folder": "Valve",
        "title": "Bistable valve (double solenoid)",
        "plc_to_sim": {
            "V_YH": ("Boolean", "Valve command high"),
            "V_YL": ("Boolean", "Valve command low"),
        },
        "sim_to_plc": {
            "V_XGH": ("Boolean", "Valve is high"),
            "V_XGL": ("Boolean", "Valve is low"),
        },
    },
    "transport": {
        "folder": "Transport",
        "title": "Conveyor sorting with four ejectors",
        "plc_to_sim": {
            "T_YH11": ("Boolean", "Start transport 1 motor"),
            "T_YH12": ("Boolean", "Start transport 2 motor"),
            "T_YH13": ("Boolean", "Start transport 3 motor"),
            "T_YH14": ("Boolean", "Start transport 4 motor"),
            "T_YH15": ("Boolean", "Start transport 5 motor"),
            "T_YH21": ("Boolean", "Extend piston ejector 1"),
            "T_YL21": ("Boolean", "Retract piston ejector 1"),
            "T_YH22": ("Boolean", "Extend piston ejector 2"),
            "T_YL22": ("Boolean", "Retract piston ejector 2"),
            "T_YH23": ("Boolean", "Extend piston ejector 3"),
            "T_YL23": ("Boolean", "Retract piston ejector 3"),
            "T_YH24": ("Boolean", "Extend piston ejector 4"),
            "T_YL24": ("Boolean", "Retract piston ejector 4"),
            "T_YR11": ("Int16", "Motor speed transport 1 (0-32767)"),
            "T_YR12": ("Int16", "Motor speed transport 2 (0-32767)"),
            "T_YR13": ("Int16", "Motor speed transport 3 (0-32767)"),
            "T_YR14": ("Int16", "Motor speed transport 4 (0-32767)"),
            "T_YR15": ("Int16", "Motor speed transport 5 (0-32767)"),
        },
        "sim_to_plc": {
            "T_XGH11": ("Boolean", "Transport 1 motor running"),
            "T_XGH12": ("Boolean", "Transport 2 motor running"),
            "T_XGH13": ("Boolean", "Transport 3 motor running"),
            "T_XGH14": ("Boolean", "Transport 4 motor running"),
            "T_XGH15": ("Boolean", "Transport 5 motor running"),
            "T_XGH21": ("Boolean", "Piston ejector 1 extended"),
            "T_XGL21": ("Boolean", "Piston ejector 1 retracted"),
            "T_XGH22": ("Boolean", "Piston ejector 2 extended"),
            "T_XGL22": ("Boolean", "Piston ejector 2 retracted"),
            "T_XGH23": ("Boolean", "Piston ejector 3 extended"),
            "T_XGL23": ("Boolean", "Piston ejector 3 retracted"),
            "T_XGH24": ("Boolean", "Piston ejector 4 extended"),
            "T_XGL24": ("Boolean", "Piston ejector 4 retracted"),
            "T_XCQ01": ("Boolean", "Detector 1 active"),
            "T_XCQ02": ("Boolean", "Detector 2 active"),
            "T_XCQ03": ("Boolean", "Detector 3 active"),
            "T_XCQ04": ("Boolean", "Detector 4 active"),
            "T_XCQ11": ("Boolean", "Item in front of ejector 1"),
            "T_XCQ12": ("Boolean", "Item in front of ejector 2"),
            "T_XCQ13": ("Boolean", "Item in front of ejector 3"),
            "T_XCQ14": ("Boolean", "Item in front of ejector 4"),
            "T_BQ01": ("Boolean", "Item in store 1"),
            "T_BQ02": ("Boolean", "Item in store 2"),
            "T_BQ03": ("Boolean", "Item in store 3"),
            "T_BQ04": ("Boolean", "Item in store 4"),
            "T_BQ05": ("Boolean", "Item in store 5"),
            "T_XCQ": ("UInt32", "Item identification"),
            "T_X11": ("Int16", "Transport 1 velocity"),
        },
    },
    "level": {
        "folder": "Level",
        "title": "Vessel level process (SISO, order 3+)",
        "plc_to_sim": {
            "T_FCV10": ("Int16", "Vessel system input: inlet valve command"),
        },
        "sim_to_plc": {
            "T_LT10": ("Int16", "Vessel system output: level transmitter"),
            "T_LSH10": ("Boolean", "Vessel high level switch"),
            "T_LSL10": ("Boolean", "Vessel low level switch"),
            "T_LSHH10": ("Boolean", "Vessel high-high level switch"),
        },
    },
    "intersection": {
        "folder": "Intersection",
        "title": "Road intersection with traffic and pedestrian lights",
        "plc_to_sim": {
            "I_YTL1R": ("Boolean", "Red traffic light direction 1"),
            "I_YTL1Y": ("Boolean", "Yellow traffic light direction 1"),
            "I_YTL1G": ("Boolean", "Green traffic light direction 1"),
            "I_YTL2R": ("Boolean", "Red traffic light direction 2"),
            "I_YTL2Y": ("Boolean", "Yellow traffic light direction 2"),
            "I_YTL2G": ("Boolean", "Green traffic light direction 2"),
            "I_YPL1R": ("Boolean", "Red pedestrian light direction 1"),
            "I_YPL1G": ("Boolean", "Green pedestrian light direction 1"),
            "I_YPL2R": ("Boolean", "Red pedestrian light direction 2"),
            "I_YPL2G": ("Boolean", "Green pedestrian light direction 2"),
            "I_YPD1A": ("Boolean", "Pedestrian request acknowledged, direction 1"),
            "I_YPD2A": ("Boolean", "Pedestrian request acknowledged, direction 2"),
        },
        "sim_to_plc": {
            "I_XTL1R": ("Boolean", "Red traffic light direction 1 feedback (lamp lit)"),
            "I_XTL1Y": ("Boolean", "Yellow traffic light direction 1 feedback (lamp lit)"),
            "I_XTL1G": ("Boolean", "Green traffic light direction 1 feedback (lamp lit)"),
            "I_XTL2R": ("Boolean", "Red traffic light direction 2 feedback (lamp lit)"),
            "I_XTL2Y": ("Boolean", "Yellow traffic light direction 2 feedback (lamp lit)"),
            "I_XTL2G": ("Boolean", "Green traffic light direction 2 feedback (lamp lit)"),
            "I_XPL1R": ("Boolean", "Red pedestrian light direction 1 feedback (lamp lit)"),
            "I_XPL1G": ("Boolean", "Green pedestrian light direction 1 feedback (lamp lit)"),
            "I_XPL2R": ("Boolean", "Red pedestrian light direction 2 feedback (lamp lit)"),
            "I_XPL2G": ("Boolean", "Green pedestrian light direction 2 feedback (lamp lit)"),
            "I_XTD11": ("Boolean", "Traffic detector 1 direction 1 active"),
            "I_XTD12": ("Boolean", "Traffic detector 2 direction 1 active"),
            "I_XTD21": ("Boolean", "Traffic detector 1 direction 2 active"),
            "I_XTD22": ("Boolean", "Traffic detector 2 direction 2 active"),
            "I_XPD1": ("Boolean", "Pedestrian push button direction 1"),
            "I_XPD2": ("Boolean", "Pedestrian push button direction 2"),
            "I_XF": ("Boolean", "Intersection fault (hardware error of the traffic light system, e.g. broken lamp)"),
        },
    },
}


def all_tags() -> dict:
    """tag -> (sim, direction, type, description) over all simulations; raises on duplicates."""
    tags: dict = {}
    for sim, spec in MANIFEST.items():
        for direction in ("plc_to_sim", "sim_to_plc"):
            for tag, (vt, descr) in spec[direction].items():
                if tag in tags:
                    raise ValueError("duplicate tag %s (%s and %s)" % (tag, tags[tag][0], sim))
                tags[tag] = (sim, direction, vt, descr)
    return tags
