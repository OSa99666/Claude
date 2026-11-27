"""Subscription handling: logging of client requests and spec-conformant ModifySubscription."""
import asyncio
import logging
import time

from asyncua import Client, ua

from ballplate import config
from ballplate.sim import Simulator

EP, CMD_PORT = "opc.tcp://127.0.0.1:48556", 49872


class H:
    def __init__(self):
        self.n = 0

    def datachange_notification(self, node, val, data):
        self.n += 1


async def scenario(caplog):
    cfg = config.load()
    cfg["opcua"]["mode"] = "standalone"                 # own server on a test port (the hub mode is tested in plc-hub)
    cfg["opcua"].update(endpoint=EP, publishing_interval_cap_ms=0)   # cap off: honour the client
    cfg["visual"].update(enabled=False, command_port=CMD_PORT)
    sim = Simulator(cfg)
    task = asyncio.create_task(sim.run())
    for _ in range(100):
        try:
            c = Client(EP)
            await c.connect()
            break
        except Exception:
            await asyncio.sleep(0.2)
    try:
        idx = await c.get_namespace_index(cfg["opcua"]["namespace_uri"])
        node = c.get_node(ua.NodeId("F_XG12", idx))
        sim.model.reset(-0.9, 0.0)
        sim.model.hold_vx = 0.3                       # ball moves steadily: value changes every cycle
        h = H()
        # like a PLC client: create with a 1000 ms default ...
        sub = await c.create_subscription(1000, h)
        await sub.subscribe_data_change(node, queuesize=1, sampling_interval=200)
        await asyncio.sleep(0.3)
        h.n = 0
        await asyncio.sleep(2.0)
        slow = h.n / 2.0
        # ... then modify to a fast update rate
        p = ua.ModifySubscriptionParameters()
        p.SubscriptionId = sub.subscription_id
        p.RequestedPublishingInterval = 50
        p.RequestedLifetimeCount = 10000
        p.RequestedMaxKeepAliveCount = 3000
        res = await c.uaclient.modify_subscription(p)
        assert res.RevisedPublishingInterval == 50
        await asyncio.sleep(0.3)
        h.n = 0
        t0 = time.perf_counter()
        await asyncio.sleep(1.0)
        fast = h.n / (time.perf_counter() - t0)
        return slow, fast
    finally:
        await c.disconnect()
        task.cancel()
        try:
            await task
        except asyncio.CancelledError:
            pass


def test_modify_subscription_is_applied_and_requests_are_logged(caplog):
    caplog.set_level(logging.INFO, logger="opcua.subs")
    slow, fast = asyncio.run(asyncio.wait_for(scenario(caplog), 60))
    assert 0.5 <= slow <= 1.6, slow            # 1000 ms default
    assert 15 <= fast <= 25, fast              # 50 ms after ModifySubscription
    text = caplog.text
    assert "CreateSubscription: client requested publishing interval 1000" in text
    assert "requested sampling interval(s) [200.0] ms" in text
    assert "ModifySubscription" in text and "requested publishing interval 50" in text


async def capped_scenario():
    cfg = config.load()
    cfg["opcua"]["mode"] = "standalone"                 # own server on a test port (the hub mode is tested in plc-hub)
    cfg["opcua"].update(endpoint="opc.tcp://127.0.0.1:48557", publishing_interval_cap_ms=20)
    cfg["visual"].update(enabled=False, command_port=49873)
    sim = Simulator(cfg)
    task = asyncio.create_task(sim.run())
    for _ in range(100):
        try:
            c = Client("opc.tcp://127.0.0.1:48557")
            await c.connect()
            break
        except Exception:
            await asyncio.sleep(0.2)
    try:
        idx = await c.get_namespace_index(cfg["opcua"]["namespace_uri"])
        sim.model.reset(-0.9, 0.0)
        sim.model.hold_vx = 0.3
        h = H()
        sub = await c.create_subscription(1000, h)       # PLC-style: asks for 1 s ...
        await sub.subscribe_data_change(c.get_node(ua.NodeId("F_XG12", idx)), queuesize=1, sampling_interval=1000)
        await asyncio.sleep(0.3)
        h.n = 0
        await asyncio.sleep(1.5)
        return h.n / 1.5
    finally:
        await c.disconnect()
        task.cancel()
        try:
            await task
        except asyncio.CancelledError:
            pass


def test_publishing_interval_cap_serves_slow_clients_faster(caplog):
    caplog.set_level(logging.INFO, logger="opcua.subs")
    rate = asyncio.run(asyncio.wait_for(capped_scenario(), 60))
    assert 40 <= rate <= 55, rate                         # 20 ms cap -> ~50/s instead of 1/s
    assert "client requested publishing interval 1000 ms" in caplog.text
    assert "granted 20 ms" in caplog.text and "publishing_interval_cap_ms in effect" in caplog.text
