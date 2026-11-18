"""Receives UI parameters/commands from Godot over UDP (JSON).

Godot resends the full state a few times per second, so lost packets do not
matter.  One-shot commands use a sequence counter (``center_seq``): the
simulator acts when the counter changes.
"""
from __future__ import annotations

import asyncio
import json
import logging

log = logging.getLogger("ui")


class UiLink(asyncio.DatagramProtocol):
    def __init__(self):
        self.latest: dict = {}
        self.last_seq = None
        self.centre_requested = False

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
        seq = msg.get("center_seq")
        if seq is not None:
            # only an *increase* is a button press; a restarted Godot starts again at 0
            if self.last_seq is not None and seq > self.last_seq:
                self.centre_requested = True
            self.last_seq = seq

    def take_centre(self) -> bool:
        r, self.centre_requested = self.centre_requested, False
        return r
