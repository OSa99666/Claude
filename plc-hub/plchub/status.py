"""Print the hub diagnostics:  python -m plchub.status   (hub running? which simulations are alive? PLC clients? tag quality)"""
from __future__ import annotations

import asyncio

from asyncua import Client, ua

from . import config
from .manifest import MANIFEST


async def main():
    cfg = config.load()
    c = Client(config.url(cfg), timeout=3)
    try:
        await c.connect()
    except Exception as e:
        print("hub not running on %s (%s)" % (config.url(cfg), type(e).__name__))
        return
    try:
        idx = await c.get_namespace_index(cfg["namespace_uri"])
        didx = await c.get_namespace_index(cfg["diag_namespace_uri"])
        n = await c.get_node(ua.NodeId("PlcClients", didx)).read_value()
        print("hub on %s   namespaces ns=%d / diagnostics ns=%d   PLC clients: %d" % (config.url(cfg), idx, didx, n))
        for sim, spec in MANIFEST.items():
            alive = await c.get_node(ua.NodeId(sim + ".Alive", didx)).read_value()
            tag = next(iter(spec["sim_to_plc"]))
            dv = await c.get_node(ua.NodeId(tag, idx)).read_attribute(ua.AttributeIds.Value, None, False)
            print("  %-13s %-5s  %2d tags   quality of %s: %s" % (sim, "ALIVE" if alive else "gone", len(spec["plc_to_sim"]) + len(spec["sim_to_plc"]),
                  tag, "good" if dv.StatusCode.is_good() else dv.StatusCode.name))
    finally:
        await c.disconnect()


if __name__ == "__main__":
    asyncio.run(main())
