"""The hub as a separate process: lifecycle, shared namespace, quality of dead simulations, PLC client detection."""
import asyncio
import time

from asyncua import Client, ua

from plchub import config, launcher
from plchub.client import HubLink
from plchub.manifest import MANIFEST, all_tags

from conftest import TEST_PORT

CFG = {"port": TEST_PORT, "stop_delay_s": 1.5, "heartbeat_timeout_s": 0.6, "log_file": "logs/test-hub.log"}


def link(sim):
    return HubLink(sim, CFG, io_interval_s=0.01, heartbeat_s=0.1)


async def plc():
    c = Client(config.url(config.merge(CFG)))
    await c.connect()
    idx = await c.get_namespace_index(config.merge(CFG)["namespace_uri"])
    return c, idx


async def dv(c, idx, tag):
    return await c.get_node(ua.NodeId(tag, idx)).read_attribute(ua.AttributeIds.Value, None, False)


async def wait_for(cond, timeout=5.0, step=0.05):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        r = await cond()
        if r:
            return r
        await asyncio.sleep(step)
    raise AssertionError("condition not met in %.1f s" % timeout)


def test_first_simulation_starts_hub_all_tags_in_ns2_default_and_bad_until_their_simulation_runs():
    async def scenario():
        assert not launcher.port_open(TEST_PORT)
        v = link("valve")
        await v.start()
        assert v.connected and launcher.port_open(TEST_PORT)
        c, idx = await plc()
        try:
            assert idx == 2                                                  # all tags in ns=2
            for tag in all_tags():                                           # every tag exists, whichever simulation runs
                await dv(c, idx, tag)
            other = await dv(c, idx, "F_XG10")                               # ballplate is not running
            assert other.StatusCode.is_bad() and other.Value.Value == 0
            await v.nodes["V_XGH"].write_value(ua.DataValue(ua.Variant(True, ua.VariantType.Boolean)))
            await wait_for(lambda: _is_true(c, idx, "V_XGH"))
            d = await dv(c, idx, "V_XGH")
            assert d.StatusCode.is_good() and d.Value.Value is True          # valve runs: good quality
            await v.stop()
        finally:
            await c.disconnect()
    asyncio.run(scenario())


async def _is_true(c, idx, tag):
    d = await dv(c, idx, tag)
    return d.StatusCode.is_good() and d.Value.Value is True


def test_data_both_ways_plc_connected_and_plc_vs_simulation_sessions():
    async def scenario():
        s = link("level")
        await s.start()
        await asyncio.sleep(0.5)
        assert s.connected and not s.plc_connected()                         # a simulation is not a PLC client
        c, idx = await plc()
        try:
            await wait_for(lambda: _true(s.plc_connected))
            await c.get_node(ua.NodeId("T_FCV10", idx)).write_value(ua.DataValue(ua.Variant(12345, ua.VariantType.Int16)))
            assert await wait_for(lambda: _eq(lambda: s.values["T_FCV10"], 12345))
            assert int(await s.nodes["T_FCV10"].read_value()) == 12345
            await s.nodes["T_LT10"].write_value(ua.DataValue(ua.Variant(777, ua.VariantType.Int16)))
            await s.nodes["T_LSH10"].write_value(ua.DataValue(ua.Variant(True, ua.VariantType.Boolean)))
            await wait_for(lambda: _is_true(c, idx, "T_LSH10"))
            assert (await dv(c, idx, "T_LT10")).Value.Value == 777
        finally:
            await c.disconnect()
        await wait_for(lambda: _true(lambda: not s.plc_connected()))          # PLC gone
        await s.stop()
    asyncio.run(scenario())


async def _true(f):
    return bool(f())


async def _eq(f, v):
    return f() == v


def test_dead_simulation_tags_go_to_default_and_bad_quality_and_come_back():
    async def scenario():
        s = link("intersection")
        await s.start()
        c, idx = await plc()
        try:
            await s.nodes["I_XTL1G"].write_value(ua.DataValue(ua.Variant(True, ua.VariantType.Boolean)))
            await wait_for(lambda: _is_true(c, idx, "I_XTL1G"))
            await s.stop()                                                   # the simulation dies
            async def bad():
                d = await dv(c, idx, "I_XTL1G")
                return d.StatusCode.is_bad() and d.Value.Value is False
            await wait_for(bad)                                              # default value + bad quality
            s2 = link("intersection")                                        # restarted: it rewrites what it writes
            await s2.start()
            await s2.nodes["I_XTL1G"].write_value(ua.DataValue(ua.Variant(True, ua.VariantType.Boolean)))
            await wait_for(lambda: _is_true(c, idx, "I_XTL1G"))
            await s2.stop()
        finally:
            await c.disconnect()
    asyncio.run(scenario())


def test_hub_stops_after_the_delay_when_the_last_simulation_has_stopped_and_two_simulations_share_one_hub():
    async def scenario():
        a, b = link("valve"), link("level")
        await asyncio.gather(a.start(), b.start())                           # started at the same moment: one hub
        assert a.connected and b.connected
        out = __import__("subprocess").run(["pgrep", "-fc", "plchub.hub --port %d" % TEST_PORT], capture_output=True, text=True).stdout.strip()
        assert out == "1"
        await a.stop()
        await asyncio.sleep(2.5)                                             # longer than the stop delay: b still runs -> hub stays
        assert launcher.port_open(TEST_PORT) and b.connected
        await b.stop()
        t0 = time.monotonic()
        while launcher.port_open(TEST_PORT) and time.monotonic() - t0 < 8:
            await asyncio.sleep(0.2)
        assert not launcher.port_open(TEST_PORT)                             # stopped by itself ...
        assert time.monotonic() - t0 >= 0.8                                  # ... after the delay, not at once
    asyncio.run(scenario())


def test_hub_restart_is_survived_values_are_rewritten():
    async def scenario():
        s = link("valve")
        await s.start()
        await s.nodes["V_XGL"].write_value(ua.DataValue(ua.Variant(True, ua.VariantType.Boolean)))
        await asyncio.sleep(0.3)
        __import__("subprocess").run(["pkill", "-KILL", "-f", "plchub.hub --port %d" % TEST_PORT])   # the hub dies
        await wait_for(lambda: _true(lambda: not s.connected))
        await wait_for(lambda: _true(lambda: s.connected), timeout=20)       # the link starts a new hub and reconnects
        c, idx = await plc()
        try:
            await wait_for(lambda: _is_true(c, idx, "V_XGL"))                # value written before the crash is back
        finally:
            await c.disconnect()
        await s.stop()
    asyncio.run(scenario())


def test_foreign_server_on_the_port_is_not_mistaken_for_the_hub():
    async def scenario():
        from asyncua import Server
        srv = Server()
        await srv.init()
        srv.set_endpoint("opc.tcp://0.0.0.0:%d" % TEST_PORT)
        await srv.start()
        try:
            try:
                await launcher.ensure_hub(config.merge(CFG))
                raise AssertionError("should refuse")
            except RuntimeError as e:
                assert "not the PLC simulation hub" in str(e)
        finally:
            await srv.stop()
    asyncio.run(scenario())
