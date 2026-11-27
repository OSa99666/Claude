"""Real OPC UA round trip (standalone mode): asyncua client <-> Simulator server, plus the Godot UI link."""
import asyncio
import json
import socket

from asyncua import Client, ua

from jacketsim import config
from jacketsim.sim import Simulator

EP, CMD_PORT = "opc.tcp://127.0.0.1:48559", 49881


def send_ui(**kw):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.sendto(json.dumps(kw).encode(), ("127.0.0.1", CMD_PORT))
    s.close()


async def scenario():
    cfg = config.load()
    cfg["opcua"]["mode"] = "standalone"                 # own server on a test port (the hub mode is tested in plc-hub)
    cfg["opcua"]["endpoint"] = EP
    cfg["visual"].update(enabled=False, command_port=CMD_PORT)
    cfg["sim"]["seed"] = 1
    cfg["model"]["sample_time_s"] = 0.1                 # fast plant for the test
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
        I = lambda v: ua.DataValue(ua.Variant(v, ua.VariantType.Int16))
        for n in ("M_YR01", "M_YR02", "M_XG01", "M_XG02"):
            assert await N(n).read_data_type_as_variant_type() == ua.VariantType.Int16
        rd = lambda n: N(n).read_value()

        await asyncio.sleep(0.4)
        assert abs(await rd("M_XG02") - 16384) <= 6000                  # 0.5 m at the start (the level falls a little: no inflow yet)
        assert 10000 < await rd("M_XG01") <= 16384 + 400                # 70 degC at the start, cooling while the PLC gives nothing
        # PLC drives the heater full on: the temperature rises, the level stays (inflow 0.25 -> 0.5 m)
        await N("M_YR01").write_value(I(32767))
        await N("M_YR02").write_value(I(8192))
        await asyncio.sleep(3.0)
        assert await rd("M_XG01") > 21000
        assert abs(await rd("M_XG02") - 16384) < 1500
        # raise the inflow: the level rises
        await N("M_YR02").write_value(I(20000))
        await asyncio.sleep(3.0)
        assert await rd("M_XG02") > 17500
        # UI: reset the states, manual override (heater off, inflow 0) overrides the PLC command
        send_ui(reset_seq=0, manual_on=False)
        await asyncio.sleep(0.2)
        send_ui(reset_seq=1, manual_on=True, manual_heater=0.0, manual_inflow=0.0)
        await asyncio.sleep(0.5)
        assert abs(await rd("M_XG01") - 16384) < 6000                   # reset to 70 degC, then slowly cooling
        await asyncio.sleep(2.5)
        assert await rd("M_XG02") < 15500                               # no inflow: the level falls although the PLC says 20000
        # UI: set the state directly and a level disturbance
        send_ui(reset_seq=1, manual_on=True, manual_heater=0.0, manual_inflow=0.0, set_state_seq=0, set_temp=100.0, set_level=0.8)
        await asyncio.sleep(0.2)
        send_ui(reset_seq=1, manual_on=True, manual_heater=0.0, manual_inflow=0.0, set_state_seq=1, set_temp=100.0, set_level=0.8)
        await asyncio.sleep(0.4)
        lvl, tmp = await rd("M_XG02"), await rd("M_XG01")
        assert 20000 < lvl < 29000, lvl                                  # 0.8 m, draining a little
        assert 15000 < tmp <= 26214 + 400, tmp                           # 100 degC, cooling
    finally:
        await c.disconnect()
        task.cancel()
        try:
            await task
        except asyncio.CancelledError:
            pass


def test_round_trip():
    asyncio.run(scenario())
