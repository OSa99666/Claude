"""Real OPC UA round trip: asyncua client <-> Simulator server, plus the Godot UI link."""
import asyncio
import json
import socket

from asyncua import Client, ua

from transportsim import config
from transportsim.io import PLC_BOOL, PLC_INT, SIM_BOOL, SIM_DWORD, SIM_INT
from transportsim.sim import Simulator
from transportsim.transport import COLOURS, Item

EP, CMD_PORT = "opc.tcp://127.0.0.1:48557", 49875


def send_ui(**kw):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.sendto(json.dumps(kw).encode(), ("127.0.0.1", CMD_PORT))
    s.close()


async def scenario():
    cfg = config.load()
    cfg["opcua"]["endpoint"] = EP
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
    else:
        raise RuntimeError("server did not start")
    try:
        idx = await c.get_namespace_index(cfg["opcua"]["namespace_uri"])
        N = lambda n: c.get_node(ua.NodeId(n, idx))
        B = lambda v: ua.DataValue(ua.Variant(v, ua.VariantType.Boolean))
        I = lambda v: ua.DataValue(ua.Variant(v, ua.VariantType.Int16))
        for n in PLC_BOOL + SIM_BOOL:
            assert await N(n).read_data_type_as_variant_type() == ua.VariantType.Boolean, n
        for n in PLC_INT + SIM_INT:
            assert await N(n).read_data_type_as_variant_type() == ua.VariantType.Int16, n
        for n in SIM_DWORD:
            assert await N(n).read_data_type_as_variant_type() == ua.VariantType.UInt32, n
        assert len(PLC_BOOL + PLC_INT + SIM_BOOL + SIM_DWORD + SIM_INT) == 46

        rd = lambda n: N(n).read_value()
        sim.model.f.xe = False
        send_ui(feed=False)
        # motor 1 with start delay, then speed word
        await N("T_YR11").write_value(I(32767))
        await N("T_YH11").write_value(B(True))
        await asyncio.sleep(0.05)
        assert not await rd("T_XGH11")
        await asyncio.sleep(0.3)
        assert await rd("T_XGH11")
        assert 3200 < await rd("T_X11") < 3350
        # an item of type 2 passes the detector: identification word + detector bit
        sim.model.t1.append(Item(2, 30.0))
        seen = set()
        for _ in range(80):
            seen.add((bool(await rd("T_XCQ02")), int(await rd("T_XCQ"))))
            await asyncio.sleep(0.02)
        assert (True, COLOURS[2]) in seen
        # ejector 1 extends: feedbacks
        assert await rd("T_XGL21") and not await rd("T_XGH21")
        await N("T_YH21").write_value(B(True))
        await asyncio.sleep(0.2)
        assert not await rd("T_XGL21") and not await rd("T_XGH21")     # moving
        await asyncio.sleep(0.7)
        assert await rd("T_XGH21") and not await rd("T_XGL21")
        # force feedbacks false from the UI
        send_ui(force_fb_false=[True, False, False, False])
        await asyncio.sleep(0.3)
        assert not await rd("T_XGH21") and not await rd("T_XGL21")
        send_ui(force_fb_false=[False] * 4)
        # item pushed into store 5 -> BQ05; UI button empties it
        sim.model.store[4] = 3
        await asyncio.sleep(0.1)
        assert await rd("T_BQ05")
        send_ui(empty_store5_seq=0)
        await asyncio.sleep(0.1)
        send_ui(empty_store5_seq=1)
        await asyncio.sleep(0.2)
        assert not await rd("T_BQ05")
    finally:
        await c.disconnect()
        task.cancel()
        try:
            await task
        except asyncio.CancelledError:
            pass


def test_round_trip():
    asyncio.run(scenario())
