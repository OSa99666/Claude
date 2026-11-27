"""Stand-in for the PLC: opens and closes the valve with a feedback handshake.

Connects as an OPC UA client, sets V_YH, waits for V_XGH, holds, then V_YL, waits for V_XGL ...
Commands are pulse-less (held until the feedback arrives), as a simple PLC sequence would do.
"""
from __future__ import annotations

import argparse
import asyncio
import logging

from asyncua import Client, ua
from plchub import config as hubcfg, mock_support

from . import config as cfgmod

mock_support.tolerate_bad_quality()      # a tag of a simulation that is not running reads 0 instead of raising
log = logging.getLogger("mock_plc")


async def run(url: str, namespace: str, hold_s: float, once: bool = False):
    async with Client(url) as c:
        idx = await c.get_namespace_index(namespace)
        N = lambda n: c.get_node(ua.NodeId(n, idx))
        B = lambda v: ua.DataValue(ua.Variant(v, ua.VariantType.Boolean))

        async def go(cmd, fb, other):
            await N(other).write_value(B(False))
            await N(cmd).write_value(B(True))
            t0 = asyncio.get_running_loop().time()
            while not await N(fb).read_value():
                await asyncio.sleep(0.02)
            log.info("%s reached after %.2f s", fb, asyncio.get_running_loop().time() - t0)
            await N(cmd).write_value(B(False))

        while True:
            await go("V_YH", "V_XGH", "V_YL")
            await asyncio.sleep(hold_s)
            await go("V_YL", "V_XGL", "V_YH")
            await asyncio.sleep(hold_s)
            if once:
                return


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--url", default=None)
    ap.add_argument("--hold", type=float, default=2.0, help="seconds to wait at each end")
    ap.add_argument("--once", action="store_true", help="one open/close cycle, then exit")
    a = ap.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(levelname)s %(message)s")
    logging.getLogger("asyncua").setLevel(logging.WARNING)
    cfg = cfgmod.load()
    url0, ns_uri = hubcfg.plc_target(cfg)
    url = a.url or url0
    try:
        asyncio.run(run(url, ns_uri, a.hold, a.once))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
