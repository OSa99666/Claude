"""Real OPC UA round trip: asyncua client <-> Simulator server, plus the Godot UI link."""
import asyncio
import json
import socket

from asyncua import Client, ua

from valvesim import config
from valvesim.sim import Simulator

EP, CMD_PORT = "opc.tcp://127.0.0.1:48556", 49873


def send_ui(**kw):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.sendto(json.dumps(kw).encode(), ("127.0.0.1", CMD_PORT))
    s.close()


async def scenario():
    cfg = config.load()
    cfg["opcua"]["endpoint"] = EP
    cfg["visual"].update(enabled=False, command_port=CMD_PORT)
    cfg["valve"].update(transit_to_high_s=0.6, transit_to_low_s=0.6,
                        action_delay_from_low_s=0.1, action_delay_from_high_s=0.1)
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
        for n in ("V_YH", "V_YL", "V_XGH", "V_XGL"):
            assert await N(n).read_data_type_as_variant_type() == ua.VariantType.Boolean

        async def fb():
            return bool(await N("V_XGH").read_value()), bool(await N("V_XGL").read_value())

        await asyncio.sleep(0.2)
        assert await fb() == (False, True)                 # starts Low
        await N("V_YH").write_value(B(True))
        await asyncio.sleep(0.4)
        assert await fb() == (False, False)                # travelling
        await asyncio.sleep(0.6)
        assert await fb() == (True, False)                 # High
        await N("V_YH").write_value(B(False))
        await N("V_YL").write_value(B(True))
        await asyncio.sleep(1.0)
        assert await fb() == (False, True)                 # back at Low
        await N("V_YL").write_value(B(False))
        # fault injection from the UI: force feedbacks false
        send_ui(force_false=True)
        await asyncio.sleep(0.3)
        assert await fb() == (False, False)
        send_ui(force_false=False, to_high_seq=0)
        await asyncio.sleep(0.2)
        send_ui(to_high_seq=1)                             # "Set to High" button
        await asyncio.sleep(0.3)
        assert await fb() == (True, False)
    finally:
        await c.disconnect()
        task.cancel()
        try:
            await task
        except asyncio.CancelledError:
            pass


def test_round_trip():
    asyncio.run(scenario())
