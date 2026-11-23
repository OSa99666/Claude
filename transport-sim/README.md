# Transport / sorting simulation - PLC control demo

Items are fed onto a main conveyor **T1**, identified by a detector, and pushed by four pneumatic
**ejectors** onto side conveyors **T2..T5**; every conveyor ends in a store.  The simulation is a Python port
of the library block `fbTransportSimulator`, served over OPC UA to a **PLC** (CODESYS), and shown in Godot.
It is **standalone** (own launcher, config, ports) like `../valve-sim` and `../ballplate-sim`.

```
 PLC (OPC UA client)             Linux PC
┌──────────────────┐  OPC UA   ┌─────────────────────────┐  UDP 9874 state   ┌──────────────────────┐
│ CODESYS          │──────────►│ Python simulator        │──────────────────►│ Godot: 3D view + UI  │
│ Data Source Mgr  │ port 4842 │ transport model, server │◄──────────────────│ feed, timing, forces │
└──────────────────┘           └─────────────────────────┘  UDP 9875 UI      └──────────────────────┘
```

```
 feed -> [ T1 ============== det ==== E1 === E2 === E3 === E4 ======= ]-> store 1
                                       |      |      |      |
                                      T2     T3     T4     T5   (each ends in store 2..5)
```
Item type 1..4 (blue, green, red, yellow) is meant for ejector 1..4; type 5 (grey, "untreated") has no
detector bit and runs to store 1.

## IO (OPC UA, namespace `urn:transportsim:simulation`, folder `Transport`, string node ids, 46 variables)

| Variables | Type | Direction | Meaning |
|---|---|---|---|
| `T_YH11`..`T_YH15` | Bool | PLC → sim | start transport motor 1..5 |
| `T_YH21`/`T_YL21` .. `T_YH24`/`T_YL24` | Bool | PLC → sim | extend / retract ejector 1..4 |
| `T_YR11`..`T_YR15` | Int16 | PLC → sim | motor speed 0..32767 (see below) |
| `T_XGH11`..`T_XGH15` | Bool | sim → PLC | motor running (after the start delay) |
| `T_XGH21`/`T_XGL21` .. `T_XGH24`/`T_XGL24` | Bool | sim → PLC | ejector extended / retracted (both false while moving) |
| `T_XCQ01`..`T_XCQ04` | Bool | sim → PLC | detector: item of type 1..4 at the detector |
| `T_XCQ11`..`T_XCQ14` | Bool | sim → PLC | item in front of ejector 1..4 |
| `T_XCQ` | UInt32 (DWord) | sim → PLC | item identification: ARGB colour of the item at the detector (`FF000065` type 1, `FF006500` 2, `FF650000` 3, `FF656500` 4, `FFD0D0D0` 5), else 0 |
| `T_X11` | Int16 | sim → PLC | T1 velocity (raw scale of the library: 10 ticks per minimum period = 32767) |
| `T_BQ01`..`T_BQ05` | Bool | sim → PLC | item in store 1..5 |

Everything else of the library block (item feed `XE`/`XE01`/`XES`, empty transport/stores `RX`/`RXQ0x`,
the force inputs `FSL/FSH/FH/FL`, and the `tTransportSimulatorPef` timing options) is set in the Godot panel
(tabs *Items*, *Timing*, *Forces*; defaults in `config/transport.toml`).

## Behaviour (from the library)

* **Speed:** every conveyor adds one tick per animation period (60 ticks = one item).  The period runs from
  15 ms (`YR` = 32767) to 80 ms (`YR` = 0), linear in `YR`, so a running conveyor never stops at `YR` = 0.
  It only moves while its motor emulator reports *running* (start delay 100 ms, stop delay 0).
* **Feed:** with `XE` (or a single item `XES`) a random item enters T1 when there is room (more than 90 ticks
  behind the last item): types 1..4 with 20 % each, otherwise type 1, or type 5 if "untreated items" is on.
* **Detector** at slot 2 sets `XCQ0n` for type n while the item is within ±½ item of it, and `T_XCQ` for ±¼.
* **Ejectors:** stroke 200 ms, action delay 100 ms before extending and **1 s before retracting**.  The moment an
  ejector starts to move, the item in front of it is moved onto the side conveyor.  An extended or moving
  ejector is a barrier: items queue up in front of it, one item apart.
* **Side conveyors** carry 5 items; items rolling off the end of T1..T5 land in store 1..5 (`BQ0n`) until emptied.
* `FSL1n` force-stops a motor; `FSH/FSL2n` force an ejector extended/retracted; `FH/FL2n` force both ejector
  feedbacks true/false (true wins); `FH/FL1n` force the item-in-front sensors.

Differences from the PLC library (deliberate): positions run continuously instead of in animation ticks; the
blocking logic is a proper queue (the library's per-item flag can leave stacked items overlapping once the
front one is ejected); one stroke ejects only the best-centred item; a full side conveyor pushes its oldest item
into the store.

## Quick start

```bash
./run.sh                 # simulator + mock PLC (sorts the items) + Godot; creates .venv on first run
./run.sh --no-mock       # real PLC: PLC connects to opc.tcp://<this-pc>:4842
./run.sh --no-godot      # headless
./run.sh --godot ~/godot/Godot_v4.3-stable_linux.x86_64
tools/install_desktop_launcher.sh      # desktop / app-menu icon (extra arguments go to run.sh)
pytest                                  # 8 tests incl. a real OPC UA round trip and a sorting run
```

The mock PLC (`transportsim/mock_plc.py`) tracks the item types from the detector and fires the right
ejector when the item is in front of it - a minimum example of what the CODESYS program has to do.
Connecting CODESYS: as for the rolling table (`../ballplate-sim/codesys/README.md`), with the variables above
and port 4842; `publishing_interval_cap_ms = 20` serves a client that insists on 1000 ms faster.

## Status

* Verified here: model tests (speed scaling, start delay, detector/front sensors, barrier queue, sorting over
  8 random sequences of 150 s), OPC UA round trip (types of all 46 variables, motor, detector word, ejector
  feedbacks, UI force/empty commands), headless `run.sh` with the mock PLC.
* **Not verified:** the Godot scene (no Godot in the development container) and the real PLC link.
* Open questions about `fbEucTwoStates` (not in the supplied XML): `ejector_travel_to_end` is assumed true.
