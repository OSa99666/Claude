"""Small patches for asyncua's OPC UA server subscription handling.

1. Logging: every CreateSubscription / ModifySubscription / CreateMonitoredItems request is logged
   with what the client asked for and what the server grants.  This answers "which update rate does
   the PLC's OPC UA client really request?" (see logs/sim.log, logger ``opcua.subs``).

2. ModifySubscription: asyncua ignores the requested publishing interval and keeps the one from
   CreateSubscription.  The OPC UA spec says the server must revise and apply it.  A client that
   creates a subscription with a default (often 1000 ms) and then modifies it to its configured
   update rate would otherwise stay at the default.  Here the new interval is applied by restarting
   the publish loop (monitored items are kept).

3. Optional publishing-interval cap (``[opcua] publishing_interval_cap_ms``, 0 = off): a client that
   asks for a slower interval (e.g. a PLC client that never goes below 1000 ms) is granted the cap
   instead.  The spec allows the server to revise the interval; keep-alive and lifetime counts are
   scaled up by the same factor so the subscription keeps its time-outs in seconds.  The client then
   simply receives notifications more often.

Data changes are pushed when the simulator writes a value, so the effective update rate equals the
subscription's publishing interval, whatever sampling interval the client asks for.
"""
from __future__ import annotations

import asyncio
import logging
import math
import time

from asyncua import ua
from asyncua.common import utils

log = logging.getLogger("opcua.subs")
_applied = False
_cap_ms = 0.0
_tasks: set[asyncio.Task] = set()
_pub: dict[int, dict] = {}              # subscription id -> publish statistics since the last snapshot()


def snapshot() -> list[str]:
    """Publish statistics per subscription since the last call (and reset them), as log-ready strings.

    published = notifications sent to the client; starved = publish cycles in which data was waiting but the
    client had no PublishRequest outstanding (the client is the bottleneck, not the server);
    max gap = longest time between two notifications sent."""
    out = []
    for sid, d in sorted(_pub.items()):
        out.append("subscription %d: %d notifications sent, %d publish cycles starved (no PublishRequest from the client), "
                   "longest gap between notifications %.0f ms, values waiting now %d"
                   % (sid, d["pub"], d["starved"], 1000 * d["max_gap"], d["waiting"]))
        d.update(pub=0, starved=0, max_gap=0.0)
    return out


def _restart_publish_loop(sub) -> None:
    async def go():
        old = sub._task
        sub._closing = True
        if old:
            old.cancel()
            try:
                await old
            except asyncio.CancelledError:
                pass
        sub._closing = False
        sub._task = asyncio.create_task(sub._subscription_loop())

    t = asyncio.get_running_loop().create_task(go())
    _tasks.add(t)
    t.add_done_callback(_tasks.discard)


def _patch() -> None:
    from asyncua.server.subscription_service import SubscriptionService as S

    orig_create = S.create_subscription
    orig_items = S.create_monitored_items

    async def create_subscription(self, params, callback, session_id, request_callback=None):
        asked = (params.RequestedPublishingInterval, params.RequestedMaxKeepAliveCount,
                 params.RequestedLifetimeCount)
        if _cap_ms > 0 and params.RequestedPublishingInterval > _cap_ms:
            k = params.RequestedPublishingInterval / _cap_ms     # keep time-outs unchanged in seconds
            params.RequestedPublishingInterval = _cap_ms
            params.RequestedMaxKeepAliveCount = int(math.ceil(params.RequestedMaxKeepAliveCount * k))
            params.RequestedLifetimeCount = int(math.ceil(params.RequestedLifetimeCount * k))
        res = await orig_create(self, params, callback, session_id, request_callback)
        capped = " (publishing_interval_cap_ms in effect)" if res.RevisedPublishingInterval < asked[0] else ""
        log.info("CreateSubscription: client requested publishing interval %g ms (keep-alive %d, lifetime %d)"
                 " -> granted %g ms (keep-alive %d, lifetime %d)%s, subscription id %d",
                 asked[0], asked[1], asked[2], res.RevisedPublishingInterval,
                 res.RevisedMaxKeepAliveCount, res.RevisedLifetimeCount, capped, res.SubscriptionId)
        return res

    async def create_monitored_items(self, params):
        items = params.ItemsToCreate
        sampling = sorted({float(i.RequestedParameters.SamplingInterval) for i in items})
        queues = sorted({int(i.RequestedParameters.QueueSize) for i in items})
        sub = self.subscriptions.get(params.SubscriptionId)
        pub = sub.data.RevisedPublishingInterval if sub else float("nan")
        log.info("CreateMonitoredItems: %d items on subscription %d, requested sampling interval(s) %s ms,"
                 " queue size(s) %s; changes are published every %g ms",
                 len(items), params.SubscriptionId, sampling, queues, pub)
        return await orig_items(self, params)

    def modify_subscription(self, params):
        sub = self.subscriptions.get(params.SubscriptionId)
        if sub is None:
            raise utils.ServiceError(ua.StatusCodes.BadSubscriptionIdInvalid)
        old = sub.data.RevisedPublishingInterval
        asked = req = float(params.RequestedPublishingInterval)
        if _cap_ms > 0 and req > _cap_ms:
            req = _cap_ms
        if req > 0 and abs(req - old) > 1e-9:
            sub.data.RevisedPublishingInterval = req
            _restart_publish_loop(sub)
        log.info("ModifySubscription id %d: client requested publishing interval %g ms (was %g ms) -> now %g ms",
                 params.SubscriptionId, asked, old, sub.data.RevisedPublishingInterval)
        result = ua.ModifySubscriptionResult()
        result.RevisedPublishingInterval = sub.data.RevisedPublishingInterval
        result.RevisedLifetimeCount = sub.data.RevisedLifetimeCount
        result.RevisedMaxKeepAliveCount = sub.data.RevisedMaxKeepAliveCount
        return result

    from asyncua.server.internal_subscription import InternalSubscription as IS
    orig_publish = IS.publish_results

    async def publish_results(self, requestdata=None):
        sid = self.data.SubscriptionId
        d = _pub.setdefault(sid, {"pub": 0, "starved": 0, "max_gap": 0.0, "last": None, "waiting": 0})
        before = self._publish_cycles_count
        ok = await orig_publish(self, requestdata)
        now = time.monotonic()
        if ok:
            d["pub"] += 1
            if d["last"] is not None:
                d["max_gap"] = max(d["max_gap"], now - d["last"])
            d["last"] = now
        elif self._publish_cycles_count > before:
            d["starved"] += 1
        d["waiting"] = sum(len(v) for v in self._triggered_datachanges.values())
        return ok

    IS.publish_results = publish_results
    S.create_subscription = create_subscription
    S.create_monitored_items = create_monitored_items
    S.modify_subscription = modify_subscription


def apply(publishing_interval_cap_ms: float = 0.0) -> None:
    global _applied, _cap_ms
    _cap_ms = max(0.0, float(publishing_interval_cap_ms or 0.0))
    if _applied:
        return
    try:
        _patch()
        _applied = True
    except Exception as e:  # noqa: BLE001 - never stop the simulator over diagnostics
        log.warning("could not install subscription logging/modify patch (asyncua version?): %r", e)
