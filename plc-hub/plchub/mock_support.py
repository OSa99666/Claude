"""Helper for the mock PLCs (not for the simulations): a PLC reads tags whose simulation is not running as bad quality.
A real PLC (CODESYS) just sees the quality; the simple mock PLCs would crash on it, so they read 0 / False instead."""
from __future__ import annotations


def tolerate_bad_quality() -> None:
    from asyncua.common.node import Node
    from asyncua.ua.uaerrors import BadNoCommunication
    if getattr(Node, "_plchub_tolerant", False):
        return
    orig = Node.read_value

    async def read_value(self):
        try:
            return await orig(self)
        except BadNoCommunication:
            return 0

    Node.read_value = read_value
    Node._plchub_tolerant = True
