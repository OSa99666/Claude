"""Real OPC UA round trip: asyncua client <-> Simulator server, plus the Godot UI link."""
import asyncio
import json
import socket

from asyncua import Client, ua

from levelsim import config
from levelsim.sim import Simulator

EP, CMD_PORT = "opc.tcp://127.0.0.1:48557", 49877


def send_ui(**kw):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.sendto(json.dumps(kw).encode(), ("127.0.0.1", CMD_PORT))
    s.close()


async def scenario():
    cfg = config.load()
    cfg["opcua"]["endpoint"] = EP
    cfg["visual"].update(enabled=False, command_port=CMD_PORT)
    cfg["sim"]["seed"] = 1
    cfg["process"].update(delay_s=0.0, outlet_pct=0.0, start_level=15.0, fn_hz=1.0, zeta=0.7, k_acc=0.5)
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
        for n in ("T_FCV10", "T_LT10"):
            assert await N(n).read_data_type_as_variant_type() == ua.VariantType.Int16
        for n in ("T_LSH10", "T_LSL10"):
            assert await N(n).read_data_type_as_variant_type() == ua.VariantType.Boolean

        async def lt():
            return int(await N("T_LT10").read_value())

        await asyncio.sleep(0.3)
        assert abs(await lt() - 4800) < 60                              # 15 % of 32000
        assert await N("T_LSL10").read_value() is True                  # below the low mark (20 %)
        assert await N("T_LSH10").read_value() is False
        await N("T_FCV10").write_value(ua.DataValue(ua.Variant(32000, ua.VariantType.Int16)))   # valve 100 %
        await asyncio.sleep(2.5)
        v = await lt()
        assert v > 4800 + 5000                                          # level rises
        assert await N("T_LSL10").read_value() is False                 # left the low mark
        await asyncio.sleep(2.5)
        assert await lt() >= 32000 - 100 and await N("T_LSH10").read_value() is True   # full
        await N("T_FCV10").write_value(ua.DataValue(ua.Variant(0, ua.VariantType.Int16)))
        await asyncio.sleep(2.5)                                        # inflow dies away (2nd-order block)
        # UI: manual override and set level
        send_ui(manual_on=True, manual_u=0.0, set_level_seq=0, set_level=50.0)
        await asyncio.sleep(0.2)
        send_ui(manual_on=True, manual_u=0.0, set_level_seq=1, set_level=50.0)
        await asyncio.sleep(0.3)
        assert abs(await lt() - 16000) < 400
        assert await N("T_LSH10").read_value() is False
        # manual valve overrides the PLC command
        send_ui(manual_on=True, manual_u=100.0, set_level_seq=1, set_level=50.0)
        await asyncio.sleep(1.5)
        assert await lt() > 16000 + 2000
    finally:
        await c.disconnect()
        task.cancel()
        try:
            await task
        except asyncio.CancelledError:
            pass


def test_round_trip():
    asyncio.run(scenario())
