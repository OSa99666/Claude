"""Small patches for asyncua's OPC UA server subscription handling.

1. Logging: every CreateSubscription / ModifySubscription / CreateMonitoredItems request is logged
   with what the client asked for and what the server grants.  This answers "which update rate does
   the PLC's OPC UA client really request?" (see logs/sim.log, logger ``opcua.subs``).

2. ModifySubscription: asyncua ignores the requested publishing interval and keeps the one from
   CreateSubscription.  The OPC UA spec says the server must revise and apply it.  A client that
   creates a subscription with a default (often 1000 ms) and then modifies it to its configured
   update rate would otherwise stay at the default.  Here the new interval is applied by restarting
   the publish loop (monitored items are kept).

Data changes are pushed when the simulator writes a value, so the effective update rate equals the
subscription's publishing interval, whatever sampling interval the client asks for.
"""
from __future__ import annotations

import asyncio
import logging

from asyncua import ua
from asyncua.common import utils

log = logging.getLogger("opcua.subs")
_applied = False
_tasks: set[asyncio.Task] = set()


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
        res = await orig_create(self, params, callback, session_id, request_callback)
        log.info("CreateSubscription: client requested publishing interval %g ms (keep-alive %d, lifetime %d)"
                 " -> granted %g ms, subscription id %d",
                 params.RequestedPublishingInterval, params.RequestedMaxKeepAliveCount,
                 params.RequestedLifetimeCount, res.RevisedPublishingInterval, res.SubscriptionId)
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
        req = float(params.RequestedPublishingInterval)
        if req > 0 and abs(req - old) > 1e-9:
            sub.data.RevisedPublishingInterval = req
            _restart_publish_loop(sub)
        log.info("ModifySubscription id %d: client requested publishing interval %g ms (was %g ms) -> now %g ms",
                 params.SubscriptionId, req, old, sub.data.RevisedPublishingInterval)
        result = ua.ModifySubscriptionResult()
        result.RevisedPublishingInterval = sub.data.RevisedPublishingInterval
        result.RevisedLifetimeCount = sub.data.RevisedLifetimeCount
        result.RevisedMaxKeepAliveCount = sub.data.RevisedMaxKeepAliveCount
        return result

    S.create_subscription = create_subscription
    S.create_monitored_items = create_monitored_items
    S.modify_subscription = modify_subscription


def apply() -> None:
    global _applied
    if _applied:
        return
    try:
        _patch()
        _applied = True
    except Exception as e:  # noqa: BLE001 - never stop the simulator over diagnostics
        log.warning("could not install subscription logging/modify patch (asyncua version?): %r", e)
