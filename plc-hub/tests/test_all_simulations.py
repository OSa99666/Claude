"""All five real simulations in hub mode at the same time, one PLC client with ONE connection controlling all of them."""
import asyncio
import importlib

from asyncua import Client, ua

from plchub import config

from conftest import TEST_PORT

HUB = {"port": TEST_PORT, "stop_delay_s": 1.0, "heartbeat_timeout_s": 0.6, "log_file": "logs/test-hub.log"}
SIMS = [("ballplate", 49901), ("valvesim", 49902), ("transportsim", 49903), ("levelsim", 49904), ("intersectionsim", 49905)]


def make(pkg, cmd_port):
    cfgm = importlib.import_module(pkg + ".config")
    simm = importlib.import_module(pkg + ".sim")
    cfg = cfgm.load()
    cfg["opcua"]["mode"] = "hub"
    cfg["hub"] = dict(HUB)
    cfg["visual"].update(enabled=False, command_port=cmd_port)
    cfg["sim"]["seed"] = 1
    if pkg == "valvesim":
        cfg["valve"].update(transit_to_high_s=0.6, transit_to_low_s=0.6, action_delay_from_low_s=0.1, action_delay_from_high_s=0.1)
    if pkg == "levelsim":
        cfg["process"].update(delay_s=0.0, outlet_pct=0.0, start_level=15.0, fn_hz=1.0, zeta=0.7, k_acc=0.5)
    if pkg == "intersectionsim":
        for k in ("rate_w", "rate_e", "rate_n", "rate_s", "ped_rate_1", "ped_rate_2"):
            cfg["traffic"][k] = 0.0
    return simm.Simulator(cfg)


def test_one_plc_connection_controls_all_five_simulations():
    async def scenario():
        sims = [make(p, port) for p, port in SIMS]
        tasks = [asyncio.create_task(s.run()) for s in sims]
        c = Client(config.url(config.merge(HUB)))
        for _ in range(150):
            try:
                await c.connect()
                break
            except Exception:
                await asyncio.sleep(0.2)
        else:
            raise RuntimeError("hub did not start")
        try:
            idx = await c.get_namespace_index(config.merge(HUB)["namespace_uri"])
            assert idx == 2
            N = lambda t: c.get_node(ua.NodeId(t, idx))
            B = lambda v: ua.DataValue(ua.Variant(v, ua.VariantType.Boolean))
            I = lambda v: ua.DataValue(ua.Variant(v, ua.VariantType.Int16))

            async def val(t):
                d = await N(t).read_attribute(ua.AttributeIds.Value, None, False)
                return d.Value.Value if d.StatusCode.is_good() else None

            async def until(tag, pred, timeout=8.0):
                for _ in range(int(timeout / 0.1)):
                    v = await val(tag)
                    if v is not None and pred(v):
                        return v
                    await asyncio.sleep(0.1)
                raise AssertionError("%s did not reach the expected value (last %r)" % (tag, await val(tag)))

            for t in ("F_XG10", "V_XGL", "T_LT10", "T_LSL10", "I_XTL1R", "T_XGH11"):   # every simulation is up and writing
                await until(t, lambda v: True)
            # ballplate: tilt X -> acceleration / velocity of the ball
            await N("F_YR10").write_value(I(16000))
            await until("F_XG11", lambda v: v > 0)
            # valve: command High -> feedback High
            await N("V_YH").write_value(B(True))
            await until("V_XGH", lambda v: v is True)
            # transport: start motor 1 -> running
            await N("T_YH11").write_value(B(True))
            await until("T_XGH11", lambda v: v is True)
            # level: valve open -> level rises, low switch released
            start = await val("T_LT10")
            await N("T_FCV10").write_value(I(32000))
            await until("T_LT10", lambda v: v > start + 3000)
            await until("T_LSL10", lambda v: v is False)
            # intersection: green lamp command -> lamp feedback
            await N("I_YTL1G").write_value(B(True))
            await until("I_XTL1G", lambda v: v is True)
            # one PLC connection only
            plc_clients = await c.get_node(ua.NodeId("PlcClients", await c.get_namespace_index(config.merge(HUB)["diag_namespace_uri"]))).read_value()
            assert plc_clients == 1
            # kill one simulation: only ITS tags go bad, the others keep running
            tasks[3].cancel()
            await asyncio.gather(tasks[3], return_exceptions=True)
            for _ in range(60):
                d = await N("T_LT10").read_attribute(ua.AttributeIds.Value, None, False)
                if d.StatusCode.is_bad():
                    break
                await asyncio.sleep(0.1)
            assert d.StatusCode.is_bad() and d.Value.Value == 0
            assert await val("I_XTL1G") is True and await val("V_XGH") is True
        finally:
            await c.disconnect()
            for t in tasks:
                t.cancel()
            await asyncio.gather(*tasks, return_exceptions=True)
    asyncio.run(scenario())
