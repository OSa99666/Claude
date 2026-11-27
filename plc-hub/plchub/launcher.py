"""Start the hub if it is not running: the first simulation starts it as a detached process, later ones find it.

A lock file serialises the start (two simulations starting at the same moment spawn only one hub); the hub keeps running
when the simulation that started it stops, until ``stop_delay_s`` after the last simulation has stopped.
"""
from __future__ import annotations

import asyncio
import fcntl
import logging
import os
import socket
import subprocess
import sys
import tempfile
import time

from asyncua import Client

from . import config

log = logging.getLogger("hub.launcher")


def port_open(port: int) -> bool:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.settimeout(0.3)
        return s.connect_ex(("127.0.0.1", int(port))) == 0


async def is_our_hub(cfg: dict) -> bool:
    """Does the server on the port serve the hub's namespace?  (Something else on port 4840 must not be mistaken for it.)"""
    try:
        c = Client(config.url(cfg), timeout=3)
        await c.connect()
        try:
            return cfg["namespace_uri"] in await c.get_namespace_array()
        finally:
            await c.disconnect()
    except Exception:
        return False


def spawn_hub(cfg: dict) -> subprocess.Popen:
    cmd = [sys.executable, "-m", "plchub.hub", "--port", str(cfg["port"]), "--stop-delay", str(cfg["stop_delay_s"]),
           "--heartbeat-timeout", str(cfg["heartbeat_timeout_s"]), "--cap-ms", str(cfg["publishing_interval_cap_ms"])]
    if cfg.get("log_file"):
        cmd += ["--log-file", str(cfg["log_file"])]
    env = dict(os.environ)
    env["PYTHONPATH"] = str(config.ROOT) + os.pathsep + env.get("PYTHONPATH", "")
    log.info("starting the hub on port %s: %s", cfg["port"], " ".join(cmd[2:]))
    return subprocess.Popen(cmd, cwd=str(config.ROOT), env=env, start_new_session=True, close_fds=True,
                            stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


async def ensure_hub(cfg: dict, timeout: float = 25.0) -> None:
    """Return when a hub of ours accepts connections on the configured port; start one if needed."""
    port = int(cfg["port"])
    if port_open(port):
        if not await is_our_hub(cfg):
            raise RuntimeError("port %d is in use by something that is not the PLC simulation hub" % port)
        return
    lock_path = os.path.join(tempfile.gettempdir(), "plchub-%d.lock" % port)
    deadline = time.monotonic() + timeout
    with open(lock_path, "w") as lock:
        while True:                                            # wait for the lock without blocking the event loop
            try:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
                break
            except OSError:
                if time.monotonic() > deadline:
                    raise TimeoutError("could not get the hub start lock")
                await asyncio.sleep(0.1)
        try:
            if not port_open(port):                            # still nobody: we start it (and hold the lock until it is up)
                proc = spawn_hub(cfg)
                while not port_open(port):
                    if proc.poll() is not None:
                        raise RuntimeError("the hub exited at start (exit code %s); see %s" % (proc.returncode, cfg.get("log_file")))
                    if time.monotonic() > deadline:
                        raise TimeoutError("the hub did not come up on port %d" % port)
                    await asyncio.sleep(0.1)
                proc.returncode  # the hub is detached; we do not wait for it
        finally:
            fcntl.flock(lock, fcntl.LOCK_UN)
