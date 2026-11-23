"""Receives UI parameters/commands from Godot over UDP (JSON).

Godot resends the full state a few times per second, so lost packets do not matter.
One-shot commands use sequence counters (names ending in ``_seq``): the simulator acts
when a counter increases (a restarted Godot starts again at 0, which is not a press).
"""
from __future__ import annotations

import asyncio
import json
import logging

log = logging.getLogger("ui")


class UiLink(asyncio.DatagramProtocol):
    def __init__(self):
        self.latest: dict = {}
        self.last_seq: dict[str, int] = {}
        self.pending: set[str] = set()

    @classmethod
    async def start(cls, port: int, host: str = "0.0.0.0") -> "UiLink":
        loop = asyncio.get_running_loop()
        _, proto = await loop.create_datagram_endpoint(cls, local_addr=(host, port))
        log.info("listening for Godot UI commands on UDP %s:%d", host, port)
        return proto

    def datagram_received(self, data, addr):
        try:
            msg = json.loads(data.decode())
        except ValueError:
            return
        if not isinstance(msg, dict):
            return
        self.latest = msg
        for k, v in msg.items():
            if k.endswith("_seq") and isinstance(v, int):
                if k in self.last_seq and v > self.last_seq[k]:
                    self.pending.add(k)
                self.last_seq[k] = v

    def take(self, key: str) -> bool:
        if key in self.pending:
            self.pending.discard(key)
            return True
        return False
