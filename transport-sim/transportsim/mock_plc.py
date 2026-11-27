"""Stand-in for the PLC: runs all conveyors and sorts item types 1..4 with ejectors 1..4.

Item type k is pushed onto side conveyor k+1 by ejector k; type 5 ('untreated') runs to store 1.
The detector (T_XCQ != 0) tells the type of each item; items are tracked in queues until they pass
the ejectors.  Connects as an OPC UA client and polls the IO every 20 ms.
"""
from __future__ import annotations

import argparse
import asyncio
import logging
import time

from asyncua import Client, ua

from . import config as cfgmod
from .io import PLC_BOOL, PLC_INT, SIM_BOOL
from .transport import COLOURS

log = logging.getLogger("mock_plc")
TYPE_OF = {v: k for k, v in COLOURS.items()}


class Sorter:
    def __init__(self, speed=(24000, 20000, 20000, 20000, 20000)):
        self.speed = speed
        self.queue = [[] for _ in range(4)]       # per ejector: items that still have to pass it
        self.window = [None] * 4
        self.prev_xcq = 0
        self.prev_front = [False] * 4
        self.out = {n: False for n in PLC_BOOL}
        self.out.update({n: 0 for n in PLC_INT})
        self.phase = ["idle"] * 4                 # idle / extending / retracting
        self.sorted = [0] * 4
        self.wait_run = [None] * 5                # since when a motor has been commanded but is not running
        self.reset_until = [0.0] * 5              # motor trip reset: start command dropped until this time

    def update(self, i: dict, now: float | None = None) -> dict:
        now = time.monotonic() if now is None else now
        o = self.out
        for n in range(1, 6):
            o["T_YR1%d" % n] = self.speed[n - 1]
            # a motor that does not report 'running' for > 1 s has tripped: drop the start command briefly to reset it
            if now < self.reset_until[n - 1]:
                o["T_YH1%d" % n] = False
                continue
            o["T_YH1%d" % n] = True
            if i["T_XGH1%d" % n]:
                self.wait_run[n - 1] = None
            elif self.wait_run[n - 1] is None:
                self.wait_run[n - 1] = now
            elif now - self.wait_run[n - 1] > 1.0:
                self.wait_run[n - 1] = None
                self.reset_until[n - 1] = now + 0.3
                o["T_YH1%d" % n] = False
        # new item at the detector (rising edge of the identification word)
        xcq = int(i["T_XCQ"])
        if xcq and not self.prev_xcq:
            item = {"type": TYPE_OF.get(xcq, 5), "ejected": False}
            for q in self.queue:
                q.append(item)
        self.prev_xcq = xcq
        for k in range(4):
            n = k + 1
            front = bool(i["T_XCQ1%d" % n])
            if front and not self.prev_front[k]:
                while self.queue[k] and self.queue[k][0]["ejected"]:
                    self.queue[k].pop(0)
                self.window[k] = self.queue[k][0] if self.queue[k] else None
            w = self.window[k]
            if (front and w and w["type"] == n and not w["ejected"] and self.phase[k] == "idle"
                    and i["T_XGL2%d" % n]):
                o["T_YH2%d" % n], o["T_YL2%d" % n] = True, False
                w["ejected"] = True
                self.phase[k] = "extending"
                self.sorted[k] += 1
            if self.prev_front[k] and not front:
                if self.queue[k] and self.queue[k][0] is self.window[k]:
                    self.queue[k].pop(0)
                self.window[k] = None
            self.prev_front[k] = front
            if self.phase[k] == "extending" and i["T_XGH2%d" % n]:
                o["T_YH2%d" % n], o["T_YL2%d" % n] = False, True
                self.phase[k] = "retracting"
            elif self.phase[k] == "retracting" and i["T_XGL2%d" % n]:
                o["T_YL2%d" % n] = False
                self.phase[k] = "idle"
        return o


async def run(url: str, namespace: str, speed: int | None):
    sp = (24000, 20000, 20000, 20000, 20000) if speed is None else (speed,) * 5
    sorter = Sorter(sp)
    async with Client(url) as c:
        idx = await c.get_namespace_index(namespace)
        N = lambda n: c.get_node(ua.NodeId(n, idx))
        in_names = list(SIM_BOOL) + ["T_XCQ"]
        in_nodes = [N(n) for n in in_names]
        out_nodes = {n: N(n) for n in PLC_BOOL + PLC_INT}
        last = {}
        log_t = 0.0
        loop = asyncio.get_running_loop()
        while True:
            vals = await c.read_values(in_nodes)
            o = sorter.update(dict(zip(in_names, vals)))
            for n, v in o.items():
                if last.get(n) != v:
                    vt = ua.VariantType.Int16 if n in PLC_INT else ua.VariantType.Boolean
                    await out_nodes[n].write_value(ua.DataValue(ua.Variant(v, vt)))
                    last[n] = v
            if loop.time() - log_t > 5:
                log.info("sorted per ejector: %s", sorter.sorted)
                log_t = loop.time()
            await asyncio.sleep(0.02)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--url", default=None)
    ap.add_argument("--speed", type=int, default=None, help="raw speed 0..32767 for all conveyors")
    a = ap.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(levelname)s %(message)s")
    logging.getLogger("asyncua").setLevel(logging.WARNING)
    cfg = cfgmod.load()
    url = a.url or "opc.tcp://127.0.0.1:%d" % int(cfg["opcua"]["endpoint"].rsplit(":", 1)[1])
    try:
        asyncio.run(run(url, cfg["opcua"]["namespace_uri"], a.speed))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
