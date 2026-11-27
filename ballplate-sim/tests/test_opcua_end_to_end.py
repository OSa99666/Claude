"""Real OPC UA round trip: asyncua client <-> Simulator server, plus the Godot UI link."""
import asyncio
import json
import math
import socket

import pytest
from asyncua import Client, ua

from ballplate import config
from ballplate.scaling import from_raw
from ballplate.sim import Simulator

EP, CMD_PORT = "opc.tcp://127.0.0.1:48555", 49871


def send_ui(**kw):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.sendto(json.dumps(kw).encode(), ("127.0.0.1", CMD_PORT))
    s.close()


async def scenario():
    cfg = config.load()
    cfg["opcua"]["mode"] = "standalone"                 # own server on a test port (the hub mode is tested in plc-hub)
    cfg["opcua"]["endpoint"] = EP
    cfg["visual"].update(enabled=False, command_port=CMD_PORT)
    sim = Simulator(cfg)
    sim.model.reset(0.0, 0.0)
    task = asyncio.create_task(sim.run())
    for _ in range(100):                       # wait for the server
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
        I16 = lambda v: ua.DataValue(ua.Variant(v, ua.VariantType.Int16))

        async def rd(name):
            return await N(name).read_value()

        def m(raw, lim=1.0):
            return from_raw(raw, -lim, lim)

        # variables are Int16 and exactly the eight agreed names
        for n in ("F_YR10", "F_YR20", "F_XG10", "F_XG11", "F_XG12", "F_XG20", "F_XG21", "F_XG22"):
            assert await N(n).read_data_type_as_variant_type() == ua.VariantType.Int16

        # 1. +15 degrees tilt in X (raw +32000) -> ball moves to +X, Y untouched
        await N("F_YR10").write_value(I16(32000))
        await asyncio.sleep(0.7)            # ~0.45 m of travel, still moving (range-saturated)
        x = m(await rd("F_XG12"))
        y = m(await rd("F_XG22"))
        vx = from_raw(await rd("F_XG11"), -0.4, 0.4)
        assert 0.25 < x < 0.7 and abs(y) < 0.01 and vx > 0.2
        assert x == pytest.approx(sim.model.s.x, abs=0.05)
        # acceleration word is negative or positive but within range
        assert abs(await rd("F_XG10")) <= 32000

        # 2. UI: hold velocity -0.2 in X, tilt back to 0
        await N("F_YR10").write_value(I16(0))
        send_ui(hold_vx=True, vel_sp_x=-0.2, elasticity=0.6, mu_s=0.35, mu_k=0.25,
                vel_min=-0.4, vel_max=0.4, center_seq=0)
        await asyncio.sleep(0.5)
        assert from_raw(await rd("F_XG11"), -0.4, 0.4) == pytest.approx(-0.2, abs=0.01)

        # 3. UI: centre command (sequence number changes), holds off
        send_ui(center_seq=1)
        await asyncio.sleep(0.3)
        assert abs(sim.model.s.x) < 0.01 and abs(await rd("F_XG12")) < 400

        # 4. UI: hold position X = 0.5 m
        send_ui(hold_x=True, dist_sp_x=0.5, center_seq=1)
        await asyncio.sleep(0.3)
        assert m(await rd("F_XG12")) == pytest.approx(0.5, abs=0.01)
        send_ui(hold_x=False, center_seq=1)

        # 5. negative elasticity removes the walls; ball driven off the table
        send_ui(elasticity=-1.0, center_seq=1)
        await N("F_YR10").write_value(I16(32000))
        for _ in range(100):
            await asyncio.sleep(0.1)
            if sim.model.s.off_table:
                break
        assert sim.model.s.off_table
        await asyncio.sleep(0.1)
        assert [await rd(n) for n in ("F_XG10", "F_XG11", "F_XG12", "F_XG20", "F_XG21", "F_XG22")] == [0] * 6

        # 6. centre brings the ball back
        send_ui(elasticity=0.6, center_seq=2)
        await asyncio.sleep(0.3)
        assert not sim.model.s.off_table
    finally:
        await c.disconnect()
        task.cancel()
        try:
            await task
        except asyncio.CancelledError:
            pass


def test_opcua_round_trip_and_ui_commands():
    asyncio.run(asyncio.wait_for(scenario(), 60))
