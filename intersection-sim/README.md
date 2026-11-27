# Intersection simulation - PLC control demo

A road intersection with vehicles and pedestrians, simulated in Python and shown in Godot (3D).  The **PLC** (CODESYS)
controls the traffic lights and pedestrian lights over OPC UA and gets lamp feedback, vehicle detectors, pedestrian push buttons and an
intersection fault.  **Standalone**: own launcher, config and ports (OPC UA 4844, UDP 9878/9879), like the other four demos.
It re-creates the existing CODESYS intersection simulation (picture supplied by the user) with moving traffic.

```
 PLC (OPC UA client)             Linux PC
┌──────────────────┐  OPC UA   ┌─────────────────────────┐  UDP 9878 state   ┌──────────────────────┐
│ CODESYS          │──────────►│ Python simulator        │──────────────────►│ Godot: 3D view +     │
│ Data Source Mgr  │ port 4844 │ traffic model, server   │◄──────────────────│ traffic / fault UI   │
└──────────────────┘           └─────────────────────────┘  UDP 9879 UI      └──────────────────────┘
```

## IO (OPC UA, namespace `urn:intersectionsim:simulation`, folder `Intersection`, string node ids = the tag names, all Bool, 29 variables)

PLC → simulator (12):

| Tag | Meaning |
|---|---|
| `I_YTL1R` `I_YTL1Y` `I_YTL1G` | traffic light direction 1 (west-east): red / yellow / green |
| `I_YTL2R` `I_YTL2Y` `I_YTL2G` | traffic light direction 2 (north-south) |
| `I_YPL1R` `I_YPL1G` | pedestrian light direction 1 (red / green) |
| `I_YPL2R` `I_YPL2G` | pedestrian light direction 2 |
| `I_YPD1A` `I_YPD2A` | pedestrian request acknowledged (lamp at the push button; pedestrians do not press again while it is lit) |

Simulator → PLC (17):

| Tag | Meaning |
|---|---|
| `I_XTL1R/Y/G`, `I_XTL2R/Y/G` | traffic light feedback = lamp really lit (command and lamp not broken) |
| `I_XPL1R/G`, `I_XPL2R/G` | pedestrian light feedback |
| `I_XTD11` `I_XTD12` | vehicle detector direction 1: arm west / arm east (loop at the stop line, true while a car is on it) |
| `I_XTD21` `I_XTD22` | vehicle detector direction 2: arm north / arm south |
| `I_XPD1` `I_XPD2` | pedestrian push button group 1 / 2 (pulse of 0.6 s per press; a waiting pedestrian presses again after 10 s unless `I_YPDnA` is lit) |
| `I_XF` | intersection fault: error in the traffic light **hardware** - true while any lamp is broken (also forced from the panel).  A collision is *not* reported to the PLC |

**Addresses:** the OPC UA variables are identified by tag name, so the simulator does not depend on the %Q/%I addresses.  The list
supplied has overlaps (`I_YPL2R`..`I_YPD2A` start again at `%Q50.0`, `I_XPL2R`.. at `%I50.0`, `I_XF` at `%I50.0`); within the Data Source
Manager mapping they should be unique, for example:

| PLC outputs | | PLC inputs | |
|---|---|---|---|
| `I_YTL1R`..`I_YPL1G` | `%Q50.0`..`%Q50.7` | `I_XTL1R`..`I_XPL1G` | `%I50.0`..`%I50.7` |
| `I_YPL2R`, `I_YPL2G` | `%Q51.0`, `%Q51.1` | `I_XPL2R`, `I_XPL2G` | `%I51.0`, `%I51.1` |
| `I_YPD1A`, `I_YPD2A` | `%Q51.2`, `%Q51.3` | `I_XTD11`..`I_XPD2` | `%I51.2`..`%I51.7` |
| | | `I_XF` | `%I52.0` |

(The comments of `I_YPD2A` and `I_XPD2` say "direction 1" - presumably direction 2.)

## Model (`intersectionsim/model.py`)

* Two crossing roads, one lane per travel direction, straight-through traffic only.  Direction 1 = west-east (arms W, E), direction 2 =
  north-south (arms N, S).  Intersection box 8 x 8 m, stop lines 8 m from the centre, crosswalks 3 m wide on all four arms.
* **Vehicles** arrive randomly per arm (veh/min, seeded) and follow the intelligent-driver model: they queue behind each other, stop at the
  stop line on **red**, on **yellow** if they can stop comfortably (otherwise they continue), go on **green**.  A **dark head** (no lamp lit,
  also while no PLC is connected) is treated as red.  Vehicles also stop for a pedestrian on the crosswalk ahead.  A configurable share of
  **signal runners** ignores the lights.
* **Pedestrians**: group 1 walks along direction 1 and uses the crosswalks over the direction-2 road (north / south arms), so `I_YPL1G` is
  meant to be green while the direction-1 traffic light is green; group 2 the other way round.  A pedestrian presses the push button, waits and
  starts to cross when the pedestrian light is green (feedback) and no car is about to reach the crosswalk.
* **Collision** (not an IO): two vehicles of different directions overlap (e.g. both lights green), a vehicle hits a pedestrian, or a vehicle runs into the
  one in front in its own lane (rear-end: the follower stops at the rear of the car in front, no overlap, both turn red).  The vehicles stay
  as wrecks; they stay red until the collision is cleared; the crash is latched until *Clear collision* in the panel (the PLC only sees the effect on the detectors).
* Scoreboard (in the HUD): vehicles passed, average wait, queues, pedestrians crossed / average wait, red-light violations, crashes.
* Faults for the exercises (panel): broken lamps (feedback reads false, drivers see a dark lamp), detectors dead / stuck, push buttons stuck.

## Quick start

```bash
./run.sh                 # simulator + mock PLC (fixed-time two-phase signals) + Godot; creates .venv on first run
./run.sh --green 15      # mock PLC: vehicle green time per direction in s
./run.sh --no-mock       # real PLC: PLC connects to opc.tcp://<this-pc>:4844
./run.sh --no-godot      # headless
tools/install_desktop_launcher.sh      # desktop / app-menu icon (extra arguments go to run.sh)
pytest                                  # 15 tests incl. a real OPC UA round trip
```

CODESYS: same procedure as for the rolling table (`../ballplate-sim/codesys/README.md`).  The subscription time must match the
DatasourcesTasks cycle (40 ms needs a 10 ms task), otherwise the Data Source Manager falls back to 1000 ms;
`publishing_interval_cap_ms = 20` in `config/intersection.toml` serves a client that insists on 1000 ms faster.

## Status

* Verified here: model tests (stopping, queues, yellow decision, crash + latch, runners, pedestrians, faults, repeatability, fixed-time controller
  without crash), OPC UA round trip, headless `run.sh` with the mock PLC.
* **Not run in Godot** (no Godot in the development container) - the 3D view and panel have to be checked by the user.
* Assumptions (confirmed by the user): the pedestrian light / direction naming (group 1 walks with direction-1 traffic), the detector assignment
  (`I_XTD11` west arm, `I_XTD12` east, `I_XTD21` north, `I_XTD22` south), `I_XF` = hardware error (broken lamp; detector/button faults do not set it).  Not verified: the real PLC link.
