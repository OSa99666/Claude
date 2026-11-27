# plc-hub - the common OPC UA server of all simulations

One OPC UA server for **all six simulations** (`ballplate-sim`, `valve-sim`, `transport-sim`, `level-sim`, `intersection-sim`, `jacket-sim`):
**port 4840**, all 96 tags in **namespace ns=2** (`urn:plcdemo:simulation`, string node ids = the PLC tag names).  The PLC needs
**one** data source connection; the simulations are started separately, each in its own process, and talk to the hub as OPC UA
clients.

```
 PLC (CODESYS, ONE OPC UA client)        Linux PC
┌────────────────────┐  OPC UA 4840  ┌──────────────────────────┐   OPC UA (localhost)   ┌──────────────────────┐
│ Data Source Manager│──────────────►│ plc-hub  (own process)   │◄──────────────────────►│ ballplate-sim        │
└────────────────────┘               │ ns=2: 96 tags            │◄──────────────────────►│ valve-sim            │
                                     │ ns=3: diagnostics        │◄──────────────────────►│ transport-sim  ...   │
                                     └──────────────────────────┘                        └──────────────────────┘
```

## How it behaves

* **Start / stop:** the first simulation that runs starts the hub as a *detached* process (a lock file prevents two simultaneous
  starts); later ones find it.  The hub keeps running when the simulation that started it stops and exits `stop_delay_s`
  (`config/hub.toml`, default 30 s, `0` = at once) after the **last** simulation has stopped.  If the hub dies, every simulation
  restarts it and reconnects (and rewrites its values); the PLC's connection drops meanwhile.
* **Tags:** all tags exist as long as the hub runs (the PLC can always browse/import the complete list), whether the owning
  simulation runs or not.  The single list is `plchub/manifest.py` (a new simulation needs an entry there; `tests/test_manifest.py`
  checks uniqueness and that every simulation's own tag lists match).
* **Dead simulation:** every simulation writes a heartbeat (about every 0.3 s).  Without heartbeat for `heartbeat_timeout_s`
  (1 s) the simulation counts as dead: its sim -> PLC tags go to their **default value (0 / false) with bad quality**
  (`BadNoCommunication`) until it is back (also before it has started).  A hung simulation loop is detected the same way.
  A PLC read of such a tag gets the bad status.  Observed with the CODESYS Data Source Manager (confirmed by the user): the PLC variables
  freeze at their last value (the default value is not applied) and the Data Source Manager shows that the data is not current.
* **"PLC connected":** the hub counts the OPC UA sessions that are not simulations (the simulations identify themselves by the
  session name `plchub-sim:<name>`); `Hub/PlcClients` (ns=3) is what the simulations use for their "no PLC" behaviour (e.g. the
  table levels itself).  Any other OPC UA client (UaExpert ...) counts as a PLC client too.
* **Diagnostics (ns=3, `urn:plcdemo:hub`):** `Hub/PlcClients`, `Hub/<sim>/Alive`, `Hub/<sim>/Heartbeat`.
  `python -m plchub.status` prints them; the log is `logs/hub.log` (alive simulations, PLC clients, subscription statistics).
* **Update rate:** `publishing_interval_cap_ms = 20` as before (a PLC client that asks for 1000 ms is served every 20 ms).  One data
  source connection means one subscription time / one DatasourcesTasks cycle for all tags: it must be the fastest one needed (ballplate:
  40 ms with a 10 ms task, see `../ballplate-sim/codesys/README.md`).

## Using it

* Nothing to start by hand: `./run.sh` in any simulation directory starts the hub if needed (it puts `../plc-hub` on `PYTHONPATH`).
* Real PLC: `opc.tcp://<this-pc>:4840`, namespace `urn:plcdemo:simulation` = **ns=2**.  Tag names/types/descriptions: `plchub/manifest.py`
  and the README of each simulation.
* `[opcua] mode = "standalone"` in a simulation's config gives it its own OPC UA server on `endpoint` (used by that simulation's tests
  and for development without the hub).  A simulation's config may carry a `[hub]` table (port, stop_delay_s ...) overriding `config/hub.toml`.
* Port 4840 used by something else: the launcher checks that the server there serves the hub's namespace and refuses otherwise.
* Environment: `PLCHUB_CONFIG=<file>` selects another `hub.toml`.

## Architecture notes

* `plchub/hub.py` server + watchdog + shutdown; `plchub/client.py` `HubLink`/`HubNode` (a simulation keeps its cycle: `read_value()` is
  the cached last PLC value, `write_value()` only queues; a background task does batched writes of changed values, a 10 ms subscription
  to the PLC's tags and the heartbeat, so the network never blocks the simulation cycle); `plchub/launcher.py` (find / start the hub);
  `plchub/subscription_tweaks.py` is now the **single** copy of the asyncua patches (cap, ModifySubscription, publish statistics) - the
  per-simulation copies are only used in standalone mode.
* The hub stores a dead simulation's *default value with bad quality* directly in the address space (asyncua's normal write path would
  null the value of a bad-quality data value) and uses a few asyncua internals (`iserver._external_sessions`, address space attributes);
  tests cover them - re-check after an asyncua upgrade.
* No access control: the PLC could overwrite a simulation's output tags and the other way round (as before).

## Measured (prototype, 4 cores, six simulations at 100 Hz each doing a batched exchange, one PLC subscribed at 20 ms)

Latency simulation write -> PLC notification median 9 ms / p99 13.5 ms (in-process server: 6.5 / 12.8 ms); exchange per simulation cycle
median 2-3 ms (decoupled from the cycle by `HubLink`); the hub used about 56 % of one core in that worst case (polling reads, every
output changing every cycle); five separate servers used about 42 % in total.

## Tests

`python -m pytest -q` in `plc-hub` (needs `../*-sim` on the path, done by `pytest.ini`): manifest, hub lifecycle (first start, shared hub,
stop delay, restart, foreign server on the port), data both ways, dead simulation quality, PLC vs simulation sessions, and all six real
simulations in hub mode controlled by one PLC connection.

## Status

Confirmed by the user with a real PLC: one data source connection controls all simulations in parallel through the hub; when a simulation
stops, its variables freeze and the Data Source Manager flags the data as not current (accepted).
