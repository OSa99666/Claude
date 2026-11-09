"""List the OPC UA variables under the CODESYS Application so you can copy node ids.

    python -m tools.browse_nodes opc.tcp://192.168.1.50:4840 [user password]
"""
import asyncio
import sys

from asyncua import Client


async def walk(node, depth=0, maxdepth=6):
    for ch in await node.get_children():
        name = (await ch.read_browse_name()).Name
        cls = await ch.read_node_class()
        print("  " * depth + f"{name}   [{ch.nodeid.to_string()}]")
        if depth < maxdepth and name not in ("Server",):
            await walk(ch, depth + 1, maxdepth)


async def main():
    url = sys.argv[1]
    c = Client(url)
    if len(sys.argv) > 3:
        c.set_user(sys.argv[2]); c.set_password(sys.argv[3])
    async with c:
        await walk(c.nodes.objects)


asyncio.run(main())
