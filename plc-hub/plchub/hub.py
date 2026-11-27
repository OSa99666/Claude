"""The common OPC UA server of all simulations (run as ``python -m plchub.hub``; normally started by the first simulation).

* One endpoint (default port 4840), namespace ns=2 with ALL tags of ``manifest.MANIFEST`` (string node ids = tag names), so
  the PLC uses one connection.  The tags exist while the hub runs, whether their simulation runs or not.
* Simulations connect as OPC UA clients (``client.HubLink``): they write their sim -> PLC tags, read their PLC -> sim tags and
  write a heartbeat.  Diagnostics live in namespace ns=3 (``Hub/PlcClients``, ``Hub/<sim>/Alive``, ``Hub/<sim>/Heartbeat``).
* A simulation without heartbeat for ``heartbeat_timeout_s`` is dead: its sim -> PLC tags get the default value and bad quality
  (BadNoCommunication) until it is back.  PlcClients = OPC UA sessions that are not simulations (the PLC).
* The hub stops ``stop_delay_s`` after the last simulation has stopped.
"""
from __future__ import annotations

import argparse
import asyncio
import logging
import signal
import time

from asyncua import Server, ua

from . import config, subscription_tweaks
from .manifest import MANIFEST

log = logging.getLogger("hub")

VT = {"Boolean": ua.VariantType.Boolean, "Int16": ua.VariantType.Int16, "UInt32": ua.VariantType.UInt32}
DEFAULT = {"Boolean": False, "Int16": 0, "UInt32": 0}
STARTUP_GRACE_S = 15.0          # the hub waits at least this long for the first simulation before it may stop


def _bad(vt_name: str) -> ua.DataValue:
    return ua.DataValue(ua.Variant(DEFAULT[vt_name], VT[vt_name]), ua.StatusCode(ua.StatusCodes.BadNoCommunication))


def _install_session_name_patch():
    """Remember the client's session name on the server side (asyncua does not), to tell simulations from the PLC."""
    from asyncua.server.internal_session import InternalSession
    if getattr(InternalSession, "_plchub_patched", False):
        return
    orig = InternalSession.create_session

    async def create_session(self, params, sockname=None):
        self.client_session_name = str(params.SessionName or "")
        return await orig(self, params, sockname)

    InternalSession.create_session = create_session
    InternalSession._plchub_patched = True


class Hub:
    def __init__(self, cfg: dict):
        self.cfg = cfg
        self.server: Server | None = None
        self.nodes: dict[str, object] = {}            # tag -> node (ns=2)
        self.sim_nodes: dict[str, dict] = {}          # sim -> {"alive": node, "hb": node}
        self.plc_node = None
        self.alive: dict[str, bool] = {s: False for s in MANIFEST}
        self._hb: dict[str, int] = {}
        self._hb_time: dict[str, float] = {}
        self._plc_clients = -1
        self._stop = asyncio.Event()
        self.started = time.monotonic()
        self.seen_sim = False

    async def start(self):
        subscription_tweaks.apply(float(self.cfg["publishing_interval_cap_ms"] or 0))
        _install_session_name_patch()
        s = Server()
        await s.init()
        s.set_endpoint("opc.tcp://0.0.0.0:%d" % int(self.cfg["port"]))
        s.set_server_name("PLC simulation hub")
        idx = await s.register_namespace(self.cfg["namespace_uri"])
        didx = await s.register_namespace(self.cfg["diag_namespace_uri"])
        top = await s.nodes.objects.add_folder(idx, "Simulations")
        for sim, spec in MANIFEST.items():
            folder = await top.add_folder(idx, spec["folder"])
            for direction in ("plc_to_sim", "sim_to_plc"):
                for tag, (vt, descr) in spec[direction].items():
                    n = await folder.add_variable(ua.NodeId(tag, idx), tag, DEFAULT[vt], varianttype=VT[vt])
                    n.description = descr
                    await n.set_writable()
                    self.nodes[tag] = n
        diag = await s.nodes.objects.add_folder(didx, "Hub")
        self.plc_node = await diag.add_variable(ua.NodeId("PlcClients", didx), "PlcClients", 0, varianttype=ua.VariantType.Int32)
        for sim in MANIFEST:
            f = await diag.add_folder(didx, sim)
            alive = await f.add_variable(ua.NodeId(sim + ".Alive", didx), "Alive", False, varianttype=ua.VariantType.Boolean)
            hb = await f.add_variable(ua.NodeId(sim + ".Heartbeat", didx), "Heartbeat", 0, varianttype=ua.VariantType.UInt32)
            await hb.set_writable()
            self.sim_nodes[sim] = {"alive": alive, "hb": hb}
        await s.start()
        self.server = s
        self.ns_index, self.diag_index = idx, didx
        for sim in MANIFEST:                           # nothing runs yet: default value + bad quality
            await self._mark_dead(sim)
        log.info("hub up on port %s: %d tags in ns=%d (%s), diagnostics ns=%d, stop delay %.0f s", self.cfg["port"],
                 len(self.nodes), idx, self.cfg["namespace_uri"], didx, float(self.cfg["stop_delay_s"]))

    async def stop(self):
        if self.server:
            await self.server.stop()

    async def _set_value_keep(self, node, dv: ua.DataValue):
        """Store a data value and notify the subscribers.  asyncua's normal write path would null the value of a bad-quality
        data value; the hub wants the DEFAULT value with bad quality, so it stores it directly (like the normal path otherwise)."""
        attval = self.server.iserver.aspace._nodes[node.nodeid].attributes[ua.AttributeIds.Value]
        attval.value = dv
        attval.value_callback = None
        for k, cb in list(attval.datachange_callbacks.items()):
            try:
                await cb(k, dv)
            except Exception:
                log.exception("data change callback failed")

    async def _mark_dead(self, sim: str):
        for tag, (vt, _) in MANIFEST[sim]["sim_to_plc"].items():
            await self._set_value_keep(self.nodes[tag], _bad(vt))
        await self.sim_nodes[sim]["alive"].write_value(False)

    def plc_clients(self) -> int:
        n = 0
        for sess in list(self.server.iserver._external_sessions.values()):
            if sess.state.name == "Activated" and not getattr(sess, "client_session_name", "").startswith(config.SIM_MARK):
                n += 1
        return n

    async def _watch(self):
        timeout = float(self.cfg["heartbeat_timeout_s"])
        stop_delay = float(self.cfg["stop_delay_s"])
        idle_since = self.started
        last_log = 0.0
        while not self._stop.is_set():
            now = time.monotonic()
            for sim, nodes in self.sim_nodes.items():
                hb = int(await nodes["hb"].read_value())
                if sim not in self._hb:
                    self._hb[sim] = hb                     # baseline: only a CHANGE of the counter shows a running simulation
                elif hb != self._hb[sim]:
                    self._hb[sim] = hb
                    self._hb_time[sim] = now
                alive = sim in self._hb_time and now - self._hb_time[sim] < timeout
                if alive and not self.alive[sim]:
                    log.info("simulation %s is up", sim)
                    self.seen_sim = True
                    await nodes["alive"].write_value(True)
                elif not alive and self.alive[sim]:
                    log.warning("simulation %s is gone (no heartbeat for %.1f s): its tags -> default value, bad quality", sim, timeout)
                    await self._mark_dead(sim)
                self.alive[sim] = alive
            n = self.plc_clients()
            if n != self._plc_clients:
                self._plc_clients = n
                await self.plc_node.write_value(ua.DataValue(ua.Variant(n, ua.VariantType.Int32)))
                log.info("PLC clients connected: %d", n)
            if any(self.alive.values()):
                idle_since = None
            else:
                if idle_since is None:
                    idle_since = now
                limit = stop_delay if self.seen_sim else max(stop_delay, STARTUP_GRACE_S)
                if now - idle_since >= limit:
                    log.info("no simulation running for %.0f s: stopping the hub", now - idle_since)
                    self._stop.set()
            if now - last_log > 10.0:
                last_log = now
                log.info("alive: %s   PLC clients: %d", ", ".join(s for s, a in self.alive.items() if a) or "-", self._plc_clients)
                for line in subscription_tweaks.snapshot():
                    log.info("OPC UA publish: %s", line)
            try:
                await asyncio.wait_for(self._stop.wait(), 0.2)
            except asyncio.TimeoutError:
                pass

    async def run(self):
        await self.start()
        loop = asyncio.get_running_loop()
        for sig in (signal.SIGINT, signal.SIGTERM):
            try:
                loop.add_signal_handler(sig, self._stop.set)
            except NotImplementedError:
                pass
        try:
            await self._watch()
        finally:
            await self.stop()
            log.info("hub stopped")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("-c", "--config", default=None)
    ap.add_argument("--port", type=int)
    ap.add_argument("--stop-delay", type=float, dest="stop_delay_s")
    ap.add_argument("--heartbeat-timeout", type=float, dest="heartbeat_timeout_s")
    ap.add_argument("--cap-ms", type=float, dest="publishing_interval_cap_ms")
    ap.add_argument("--log-file", dest="log_file")
    ap.add_argument("-v", "--verbose", action="store_true")
    a = ap.parse_args()
    cfg = config.load(a.config)
    for k in ("port", "stop_delay_s", "heartbeat_timeout_s", "publishing_interval_cap_ms", "log_file"):
        if getattr(a, k) is not None:
            cfg[k] = getattr(a, k)
    handlers = []
    if cfg.get("log_file"):
        path = config.ROOT / cfg["log_file"]
        path.parent.mkdir(parents=True, exist_ok=True)
        handlers.append(logging.FileHandler(path))
    else:
        handlers.append(logging.StreamHandler())
    logging.basicConfig(level=logging.DEBUG if a.verbose else logging.INFO, handlers=handlers,
                        format="%(asctime)s %(name)s %(levelname)s %(message)s")
    logging.getLogger("asyncua").setLevel(logging.INFO if a.verbose else logging.WARNING)
    try:
        asyncio.run(Hub(cfg).run())
    except OSError as e:
        logging.getLogger("hub").error("cannot start the hub: %s (is another OPC UA server using port %s?)", e, cfg["port"])
        raise SystemExit(1)


if __name__ == "__main__":
    main()
