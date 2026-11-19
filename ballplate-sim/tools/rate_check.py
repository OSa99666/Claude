"""Measure how fast OPC UA data really arrives from the simulator.

Run it from any machine that can reach the simulator (e.g. next to the PLC) and compare with what
the PLC sees:

    python -m tools.rate_check opc.tcp://<pc-ip>:4840                  # subscription, 20 ms
    python -m tools.rate_check opc.tcp://<pc-ip>:4840 --interval 1000  # what a 1 s default would give
    python -m tools.rate_check opc.tcp://<pc-ip>:4840 --mode poll      # tight read loop, round-trip times
    python -m tools.rate_check opc.tcp://<pc-ip>:4840 --mode both --duration 10

A subscription only sends a notification when a value CHANGES, so the ball has to be moving
(mock PLC with --pattern circle, or "Set velocity" in the Godot panel); with the ball at rest a
quiet result is correct, not slow.
"""
from __future__ import annotations

import argparse
import asyncio
import statistics
import time

from asyncua import Client, ua

DEFAULT_VARS = ["F_XG10", "F_XG11", "F_XG12", "F_XG20", "F_XG21", "F_XG22"]


class Collector:
    def __init__(self):
        self.stamps: dict[str, list[float]] = {}

    def datachange_notification(self, node, val, data):
        self.stamps.setdefault(str(node.nodeid.Identifier), []).append(time.perf_counter())


def report_gaps(label: str, stamps: dict[str, list[float]], duration: float):
    print(f"\n{label}")
    print(f"  {'variable':8} {'updates':>8} {'per second':>11} {'mean gap ms':>12} {'max gap ms':>11}")
    total = 0
    for name in sorted(stamps):
        s = stamps[name][1:]            # skip the initial value that is sent on subscribe
        gaps = [(b - a) * 1000 for a, b in zip(s, s[1:])]
        total += len(s)
        if gaps:
            print(f"  {name:8} {len(s):8d} {len(s) / duration:11.1f} {statistics.mean(gaps):12.1f} {max(gaps):11.1f}")
        else:
            print(f"  {name:8} {len(s):8d} {len(s) / duration:11.1f} {'-':>12} {'-':>11}")
    if total == 0:
        print("  no value changes received - is the ball moving?  (see --help)")
    return total


async def run_subscribe(c, nodes, interval, duration):
    col = Collector()
    sub = await c.create_subscription(interval, col)
    for n in nodes:
        await sub.subscribe_data_change(n, queuesize=1, sampling_interval=interval)
    print(f"subscribed to {len(nodes)} variables, requested publishing/sampling interval {interval} ms, "
          f"measuring {duration:.0f} s ...")
    await asyncio.sleep(0.5)
    col.stamps.clear()
    await asyncio.sleep(duration)
    await sub.delete()
    report_gaps(f"SUBSCRIPTION (requested {interval:g} ms)", col.stamps, duration)
    rates = [len(v) / duration for v in col.stamps.values() if len(v) > 1]
    if rates:
        r = max(rates)
        if interval > 0 and r < 0.6 * (1000.0 / interval):
            print(f"\n  -> updates arrive at about {r:.1f}/s, slower than the {1000 / interval:.1f}/s requested."
                  "\n     Either the values change less often than that (ball slow or at rest), or the server revised the interval.")
        else:
            print(f"\n  -> the server delivers what was requested ({r:.1f}/s at {interval:g} ms)."
                  "\n     If your PLC sees less, its Data Source Manager connection uses a longer"
                  "\n     publishing/sampling/poll interval (or its task cycle is slower).")


async def run_poll(c, nodes, duration):
    print(f"polling {len(nodes)} variables in a tight loop for {duration:.0f} s ...")
    rts, changes, last = [], 0, None
    t0 = time.perf_counter()
    while time.perf_counter() - t0 < duration:
        t = time.perf_counter()
        vals = await c.read_values(nodes)
        rts.append((time.perf_counter() - t) * 1000)
        if last is not None and vals != last:
            changes += 1
        last = vals
        await asyncio.sleep(0)
    n = len(rts)
    rts.sort()
    print("\nPOLLING (read of all variables in one request)")
    print(f"  reads            : {n} ({n / duration:.0f}/s)")
    print(f"  round trip ms    : mean {statistics.mean(rts):.2f}   p95 {rts[int(n * .95)]:.2f}   max {rts[-1]:.2f}")
    print(f"  data changed in  : {changes} reads ({changes / duration:.1f}/s)  <- the simulator refreshes at 100/s")


async def identify(url: str) -> bool:
    """Ask the server who it is before opening a session; helps when the wrong address is used."""
    probe = Client(url, timeout=5)
    try:
        eps = await probe.connect_and_get_server_endpoints()
    except Exception as e:  # noqa: BLE001
        print(f"cannot reach an OPC UA server at {url}: {e!r}")
        print("  check: simulator running?  address/port right?  firewall (TCP 4840)?")
        return False
    srv = eps[0].Server
    name = srv.ApplicationName.Text if srv.ApplicationName else "?"
    print(f"server at {url} says: name='{name}'  uri='{srv.ApplicationUri}'")
    for e in eps:
        tokens = ",".join(t.TokenType.name for t in (e.UserIdentityTokens or []))
        print(f"  endpoint: {e.SecurityMode.name:16} {e.SecurityPolicyUri.rsplit('#', 1)[-1]:16} login: {tokens}")
    if "ballplate" not in (srv.ApplicationUri or "").lower() and "rolling" not in name.lower():
        print("  !! this does not look like the rolling-table simulator (expected name 'Rolling table simulator').")
        print("     Wrong host or port?  Another OPC UA server (e.g. the CODESYS runtime) may be using this port.")
    return True


async def main_async(a):
    if not await identify(a.url):
        return
    c = Client(a.url, timeout=5)
    if a.user:
        c.set_user(a.user)
        c.set_password(a.password or "")
    async with c:
        idx = await c.get_namespace_index(a.uri)
        nodes = [c.get_node(ua.NodeId(n, idx)) for n in a.vars]
        print(f"connected to {a.url}, namespace {a.uri} = index {idx}")
        if a.mode in ("subscribe", "both"):
            await run_subscribe(c, nodes, a.interval, a.duration)
        if a.mode in ("poll", "both"):
            await run_poll(c, nodes, a.duration)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("url", nargs="?", default="opc.tcp://127.0.0.1:4840")
    ap.add_argument("--mode", choices=["subscribe", "poll", "both"], default="subscribe")
    ap.add_argument("--interval", type=float, default=20.0, help="requested publishing/sampling interval, ms (default 20)")
    ap.add_argument("--duration", type=float, default=5.0, help="seconds per measurement (default 5)")
    ap.add_argument("--vars", nargs="+", default=DEFAULT_VARS, help="variable names (default: the six F_XG words)")
    ap.add_argument("--uri", default="urn:ballplate:simulation", help="namespace uri of the simulator")
    ap.add_argument("--user")
    ap.add_argument("--password")
    a = ap.parse_args()
    import logging
    logging.getLogger("asyncua").setLevel(logging.ERROR)
    try:
        asyncio.run(main_async(a))
    except KeyboardInterrupt:
        pass
    except Exception as e:  # noqa: BLE001
        print(f"\nFAILED: {e!r}")
        if "ServiceUnsupported" in repr(e) or "NamespaceUri" in repr(e) or "BadNodeId" in repr(e):
            print("  The server answered but is not (or does not behave like) the rolling-table simulator.")
            print("  Check the URL; try: python -m tools.browse_nodes <url>")
        raise SystemExit(1)


if __name__ == "__main__":
    main()
