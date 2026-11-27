"""Real OPC UA round trip: asyncua client <-> Simulator server, plus the Godot UI link."""
import asyncio
import json
import socket

from asyncua import Client, ua

from intersectionsim import config
from intersectionsim.opcua_server import PLC_TO_SIM, SIM_TO_PLC
from intersectionsim.sim import Simulator

EP, CMD_PORT = "opc.tcp://127.0.0.1:48558", 49879


def send_ui(**kw):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.sendto(json.dumps(kw).encode(), ("127.0.0.1", CMD_PORT))
    s.close()


def test_tag_lists():
    assert len(PLC_TO_SIM) == 12 and len(SIM_TO_PLC) == 17
    for t in ("I_YTL1R", "I_YTL2G", "I_YPL1R", "I_YPL2G", "I_YPD1A", "I_YPD2A"):
        assert t in PLC_TO_SIM
    for t in ("I_XTL1R", "I_XTL2G", "I_XPL1G", "I_XPL2R", "I_XTD11", "I_XTD12", "I_XTD21", "I_XTD22", "I_XPD1", "I_XPD2", "I_XF"):
        assert t in SIM_TO_PLC


async def scenario():
    cfg = config.load()
    cfg["opcua"]["mode"] = "standalone"                 # own server on a test port (the hub mode is tested in plc-hub)
    cfg["opcua"]["endpoint"] = EP
    cfg["visual"].update(enabled=False, command_port=CMD_PORT)
    cfg["sim"]["seed"] = 1
    for k in ("rate_w", "rate_e", "rate_n", "rate_s", "ped_rate_1", "ped_rate_2"):
        cfg["traffic"][k] = 0.0
    sim = Simulator(cfg)
    task = asyncio.create_task(sim.run())
    for _ in range(100):
        try:
            c = Client(EP)
            await c.connect()
            break
        except Exception:
            await asyncio.sleep(0.2)
    else:
        raise RuntimeError("server did not start")
    try:
        idx = await c.get_namespace_index(cfg["opcua"]["namespace_uri"])
        N = lambda n: c.get_node(ua.NodeId(n, idx))
        for n in list(PLC_TO_SIM) + list(SIM_TO_PLC):
            assert await N(n).read_data_type_as_variant_type() == ua.VariantType.Boolean
        B = lambda v: ua.DataValue(ua.Variant(v, ua.VariantType.Boolean))
        rd = lambda n: N(n).read_value()

        await N("I_YTL1G").write_value(B(True))
        await N("I_YTL2R").write_value(B(True))
        await asyncio.sleep(0.3)
        assert await rd("I_XTL1G") is True and await rd("I_XTL2R") is True       # lamp feedback
        assert await rd("I_XTL1R") is False and await rd("I_XF") is False
        # broken lamp from the UI: commanded but not lit
        send_ui(bk_TL1G=True)
        await asyncio.sleep(0.3)
        assert await rd("I_XTL1G") is False
        assert await rd("I_XF") is True                                            # hardware fault: broken lamp
        send_ui(bk_TL1G=False, stuck_XTD22=True, stuck_PD2=True)
        await asyncio.sleep(0.3)
        assert await rd("I_XTL1G") is True and await rd("I_XTD22") is True and await rd("I_XPD2") is True
        send_ui(stuck_XTD22=False, stuck_PD2=False)
        # a car on the west arm at red waits and is detected; conflicting greens crash it
        sim.model.add_car("W", s=-15.0, v=8.0)
        await N("I_YTL1G").write_value(B(False))
        await N("I_YTL1R").write_value(B(True))
        await asyncio.sleep(4.0)
        assert await rd("I_XTD11") is True
        sim.model.add_car("N", s=-15.0, v=8.0)
        await N("I_YTL1R").write_value(B(False))
        await N("I_YTL1G").write_value(B(True))
        await N("I_YTL2R").write_value(B(False))
        await N("I_YTL2G").write_value(B(True))
        await asyncio.sleep(4.0)
        assert sim.model.crash and await rd("I_XF") is False                       # collision is not I_XF
        send_ui(clear_fault_seq=0)
        await asyncio.sleep(0.1)
        send_ui(clear_fault_seq=1)
        await asyncio.sleep(0.3)
        assert not sim.model.crash
        send_ui(hw_fault=True)
        await asyncio.sleep(0.3)
        assert await rd("I_XF") is True
    finally:
        await c.disconnect()
        task.cancel()
        try:
            await task
        except asyncio.CancelledError:
            pass


def test_round_trip():
    asyncio.run(scenario())
